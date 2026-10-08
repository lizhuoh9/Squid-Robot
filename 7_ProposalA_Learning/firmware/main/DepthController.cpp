/**********************************************************************
 * DepthController.cpp
 *
 * Proposal A depth hold (all quantities positive DOWNWARD):
 *   e      = depth - target                      (+ = too deep)
 *   a_des  = -KP*e - KD*v                        (wanted net acceleration)
 *   bias   = A_REST_REF + ALPHA*(depth - DEPTH_REF) + KI*integral(e)
 *                                                (the robot's own sinking tendency)
 *   effort = a_des - bias                        (what the pump has to add)
 *   duty   = effort / G_SINK  if effort > 0      (sink: valves closed + pump)
 *            effort / G_RISE  if effort < 0      (rise: valves open + pump)
 *********************************************************************/

#include "DepthController.h"

#include <cmath>

#include <Calibration.h>
#include <Sys.h>

namespace {
constexpr float MAX_DEPTH_CM = 100.0f;
constexpr uint8_t MANUAL_PWM = 255;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

DepthController::DepthController()
    : _holdingTarget(false),
      _targetDepthCm(0.0f),
      _depthFilt(0.0f),
      _speedCmS(0.0f),
      _accelCmS2(0.0f),
      _integ(0.0f),
      _dutyCmd(0.0f),
      _lastDir(0),
      _aDes(0.0f),
      _bias(0.0f),
      _controlOutput(0.0f),
      _outputSaturated(false),
      _manualDirection(BUOYANCY_STOP),
      _buoyancyDirection(BUOYANCY_STOP),
      _buoyancyPwm(0),
      _lastControlUpdateMs(0),
      _modWindowStartMs(0),
      _modOnUntilMs(0),
      _balancing(false),
      _balanceStartMs(0),
      _balanceEndMs(0) {}

void DepthController::begin() {
    resetAfterCalibration();
    _student.begin();
}

void DepthController::update(bool depthValid,
                             bool depthFresh,
                             float filteredDepthCm,
                             float depthSpeedCmS,
                             float depthAccelCmS2,
                             uint32_t nowMs) {
    _student.observe(_buoyancyDirection, _buoyancyPwm, nowMs);
    if (_balancing) {
        if (nowMs >= _balanceEndMs) {
            _balancing = false;
            stopBuoyancyOutput();
            return;
        }
        // Keep sending BALANCE; the Minima alternates E/F to vent the pressure.
        _buoyancyDirection = BUOYANCY_BALANCE;
        _buoyancyPwm = 0;
        return;
    }

    if (_manualDirection != BUOYANCY_STOP) {
        _controlOutput = (_manualDirection == cal::BUOY_SINK) ? 100.0f : -100.0f;
        _outputSaturated = true;
        _buoyancyDirection = _manualDirection;
        _buoyancyPwm = MANUAL_PWM;
        return;
    }

    if (!depthValid || !std::isfinite(filteredDepthCm) ||
        !std::isfinite(depthSpeedCmS) || !std::isfinite(depthAccelCmS2)) {
        _lastControlUpdateMs = 0;   // restart dt after the gap; keep the learned integral
        stopBuoyancyOutput();
        return;
    }

    _depthFilt = filteredDepthCm;
    _speedCmS = depthSpeedCmS;
    _accelCmS2 = depthAccelCmS2;

    if (!_holdingTarget) {
        _lastControlUpdateMs = 0;
        stopBuoyancyOutput();
        return;
    }

    if (_targetDepthCm < 0.0f || _depthFilt >= MAX_DEPTH_CM) {
        clearTarget();
        manualStop();
        return;
    }

    if (depthFresh) {
        computeDuty(nowMs);
    }
    applyModulatedOutput(nowMs);
}

// Runs once per new depth sample (~12 Hz).
void DepthController::computeDuty(uint32_t nowMs) {
    if (_lastControlUpdateMs == 0) {
        _lastControlUpdateMs = nowMs;
        return;
    }
    float dt = static_cast<float>(nowMs - _lastControlUpdateMs) * 0.001f;
    if (dt < 0.01f) {
        return;
    }
    if (dt > 0.20f) {
        dt = 0.20f;
    }
    _lastControlUpdateMs = nowMs;

    using namespace depth_cfg;
    const float e = _depthFilt - _targetDepthCm;
    _aDes = -KP * e - KD * _speedCmS;
    _bias = A_REST_REF + ALPHA * (_depthFilt - DEPTH_REF) + KI * _integ;
    _residual = _student.predict(e, _speedCmS, _accelCmS2, _bias, nowMs);
    const float effort = _aDes - _bias + _residual;
    float duty = (effort > 0.0f) ? effort / G_SINK : effort / G_RISE;

    const bool saturated = fabsf(duty) >= 1.0f;
    duty = clampf(duty, -1.0f, 1.0f);
    if (!saturated) {
        // Sitting too deep for a long time means the robot is heavier than the
        // feed-forward assumes: grow the heaviness estimate.
        constexpr float I_STATE_LIMIT = I_LIMIT / KI;
        _integ = clampf(_integ + e * dt, -I_STATE_LIMIT, I_STATE_LIMIT);
    }

    // Do not reverse for a tiny request: every valve change locks the pump
    // for 200 ms on the Minima and costs a valve cycle.
    if (_lastDir != 0 && ((duty > 0.0f) != (_lastDir > 0)) && fabsf(duty) < FLIP_MIN_DUTY) {
        duty = 0.0f;
    }
    if (duty != 0.0f) {
        _lastDir = (duty > 0.0f) ? 1 : -1;
    }

    _dutyCmd = duty;
    _controlOutput = duty * 100.0f;
    _outputSaturated = saturated;
}

// Runs every motion tick (5 ms).
void DepthController::applyModulatedOutput(uint32_t nowMs) {
    using namespace depth_cfg;
    const float mag = fabsf(_dutyCmd);
    float applied = 0.0f;
    if (mag >= DUTY_MIN) {
        applied = mag;
    } else if (mag > 0.0f) {
        // The pump does not run reliably below DUTY_MIN: inside each window run it
        // at DUTY_MIN for the matching fraction of the window.
        if (nowMs - _modWindowStartMs >= MOD_WINDOW_MS) {
            _modWindowStartMs = nowMs;
            _modOnUntilMs = nowMs + static_cast<uint32_t>(MOD_WINDOW_MS * mag / DUTY_MIN + 0.5f);
        }
        if (static_cast<int32_t>(_modOnUntilMs - nowMs) > 0) {
            applied = DUTY_MIN;
        }
    }

    if (applied <= 0.0f) {
        _buoyancyDirection = BUOYANCY_STOP;
        _buoyancyPwm = 0;
        return;
    }
    _buoyancyDirection = (_dutyCmd > 0.0f) ? cal::BUOY_SINK : cal::BUOY_RISE;
    const int pwm = static_cast<int>(applied * 255.0f + 0.5f);
    _buoyancyPwm = static_cast<uint8_t>(pwm > 255 ? 255 : pwm);
}

void DepthController::setTargetDepth(float targetDepthCm) {
    if (std::fabs(targetDepthCm - _targetDepthCm) > 0.1f) { _student.reset(); _residual = 0; }
    _targetDepthCm = clampf(targetDepthCm, 0.0f, MAX_DEPTH_CM);
    _holdingTarget = true;
    _manualDirection = BUOYANCY_STOP;
    // _integ is kept: it describes the robot's trim, not the old target.
}

void DepthController::holdCurrentDepth() {
    _targetDepthCm = _depthFilt;
    _holdingTarget = true;
    _manualDirection = BUOYANCY_STOP;
}

void DepthController::clearTarget() {
    _holdingTarget = false;
    _targetDepthCm = 0.0f;
    _lastDir = 0;
    _lastControlUpdateMs = 0;
    _aDes = 0.0f;
    _bias = 0.0f;
    stopBuoyancyOutput();
}

bool DepthController::isHoldingTarget() const {
    return _holdingTarget;
}

void DepthController::manualAscend() {
    clearTarget();
    _balancing = false;
    if (_manualDirection == cal::BUOY_RISE) {
        manualStop();
    } else {
        _manualDirection   = cal::BUOY_RISE;
        _buoyancyDirection = cal::BUOY_RISE;
        _buoyancyPwm       = MANUAL_PWM;
        _controlOutput     = -100.0f;
        _outputSaturated   = true;
    }
}

void DepthController::manualDescend() {
    clearTarget();
    _balancing = false;
    if (_manualDirection == cal::BUOY_SINK) {
        manualStop();
    } else {
        _manualDirection   = cal::BUOY_SINK;
        _buoyancyDirection = cal::BUOY_SINK;
        _buoyancyPwm       = MANUAL_PWM;
        _controlOutput     = 100.0f;
        _outputSaturated   = true;
    }
}

void DepthController::manualStop() {
    _manualDirection = BUOYANCY_STOP;
    stopBuoyancyOutput();
}

void DepthController::forceBalance() {
    clearTarget();
    _manualDirection = BUOYANCY_STOP;
    _balancing       = true;
    _balanceEndMs    = millis() + 5000;
    _balanceStartMs  = millis();
}

void DepthController::resetAfterCalibration() {
    _student.reset(); _residual = 0;
    _holdingTarget = false;
    _targetDepthCm = 0.0f;
    _depthFilt = 0.0f;
    _speedCmS = 0.0f;
    _accelCmS2 = 0.0f;
    _integ = 0.0f;
    _lastDir = 0;
    _aDes = 0.0f;
    _bias = 0.0f;
    _manualDirection = BUOYANCY_STOP;
    _lastControlUpdateMs = 0;
    _modWindowStartMs = 0;
    _modOnUntilMs = 0;
    _balancing = false;
    _balanceEndMs = 0;
    _balanceStartMs = 0;
    stopBuoyancyOutput();
}

float DepthController::getFilteredDepthCm() const { return _depthFilt; }
float DepthController::getSpeedCmS() const { return _speedCmS; }
float DepthController::getAccelerationCmS2() const { return _accelCmS2; }
float DepthController::getTargetDepthCm() const { return _targetDepthCm; }
float DepthController::getControlOutput() const { return _controlOutput; }
float DepthController::getPidRawOutput() const { return _aDes; }
float DepthController::getPidBaseOutput() const { return _bias; }
float DepthController::getResidualOutput() const { return _residual; }
float DepthController::getTotalOutput() const { return _controlOutput; }
bool DepthController::isOutputSaturated() const { return _outputSaturated; }
uint8_t DepthController::getBuoyancyDirection() const { return _buoyancyDirection; }
uint8_t DepthController::getBuoyancyPwm() const { return _buoyancyPwm; }
uint8_t DepthController::getManualDirection() const { return _manualDirection; }

void DepthController::stopBuoyancyOutput() {
    _student.reset(); _residual = 0;
    _dutyCmd = 0.0f;
    _controlOutput = 0.0f;
    _outputSaturated = false;
    _buoyancyDirection = BUOYANCY_STOP;
    _buoyancyPwm = 0;
}
