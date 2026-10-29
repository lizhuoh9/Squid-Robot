/**********************************************************************
 * DepthController.cpp
 *
 * Proposal A depth hold (all quantities positive DOWNWARD):
 *   e      = depth - target                      (+ = too deep)
 *   a_des  = -KP*e - KD*v                       (original PD away from target)
 *   Approaching a possible target crossing: blend in a bounded velocity
 *   reference inside a speed-dependent estimated stopping distance.
 *   bias   = A_REST_REF + ALPHA*(depth - DEPTH_REF) + KI*integral(e)
 *                                                (the robot's own sinking tendency)
 *   effort = a_des + guarded_NN_RL - bias        (what the pump has to add)
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
                             bool forwardActive,
                             uint32_t nowMs) {
    _student.observe(_buoyancyDirection, _buoyancyPwm, forwardActive, nowMs,
                     depthValid, filteredDepthCm, depthSpeedCmS);
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
    // Keep the original A controller outside the estimated stopping distance.
    // Stopping distance is a tuning heuristic, not a learned plant forecast.
    const float speedAbs = fabsf(_speedCmS);
    const float distance = fabsf(e);
    const bool approaching = e * _speedCmS < 0.0f || distance <= 0.05f;
    const float brakeDistance = BRAKE_MARGIN_CM +
        speedAbs * APPROACH_LOOKAHEAD_S +
        speedAbs * speedAbs / (2.0f * BRAKE_ACCEL_EST);
    const float brakeWeight = approaching && speedAbs >= BRAKE_SPEED_MIN
        ? clampf((brakeDistance - distance) / BRAKE_BLEND_CM, 0.0f, 1.0f)
        : 0.0f;
    const float originalAcceleration = -KP * e - KD * _speedCmS;
    const float approachError = e + APPROACH_LOOKAHEAD_S * _speedCmS;
    const float speedReference = clampf(-(KP / KD) * approachError,
                                       -APPROACH_SPEED_MAX, APPROACH_SPEED_MAX);
    const float approachAcceleration = KD * (speedReference - _speedCmS);
    _aDes = originalAcceleration + brakeWeight *
        (approachAcceleration - originalAcceleration);
    // During hold, predict the outward excursion BEFORE crossing the error
    // boundary. This is a stopping-distance heuristic, not a hard guarantee.
    // Exclude motion returning toward the target and suppress speed noise.
    const bool movingOutward = e * _speedCmS > 0.0f && speedAbs >= BRAKE_SPEED_MIN;
    const float predictedExcursion = distance +
        speedAbs * APPROACH_LOOKAHEAD_S +
        speedAbs * speedAbs / (2.0f * BRAKE_ACCEL_EST);
    const float peakWeight = movingOutward ? clampf(
        (predictedExcursion - PEAK_GUARD_START_CM) /
        (OUTWARD_GUARD_ERROR_CM - PEAK_GUARD_START_CM), 0.0f, 1.0f) : 0.0f;
    const float outwardSign = _speedCmS > 0.0f ? 1.0f : -1.0f;
    _aDes -= outwardSign * PEAK_BRAKE_EXTRA * peakWeight;
    _bias = A_REST_REF + ALPHA * (_depthFilt - DEPTH_REF) + KI * _integ;
    _residual = _student.predict(e, _speedCmS, _accelCmS2, _bias, nowMs, _depthFilt);
    _residualApplied = _student.predictionApplied();
    _rlResidual = _student.rlOutput();
    _rlResidualApplied = _student.predictionRLApplied();
    const bool needsBraking = brakeWeight > 0.0f &&
                             _speedCmS * _aDes < 0.0f;
    // While shallow, keep the previous extra-ascent guard, but allow an
    // upward NN correction when needed to brake a fast downward approach.
    const bool brakingDescent = needsBraking && _speedCmS > 0.0f;
    if (e <= 0.0f && _residual < 0.0f && !brakingDescent) {
        _residual = 0.0f;
        _residualApplied = false;
    }
    // A model correction may strengthen braking, but must not cancel the
    // base deceleration inside the target-crossing braking zone. Outside
    // the hold band, also reject a net learned acceleration away from the
    // target if already moving away (or stopped). While returning toward
    // the target, leave the pair available for the approach braking rule.
    // Gate the applied NN+RL pair together, preserving their separate model
    // bounds and allowing their full sum whenever it helps deceleration.
    const float combinedCorrection = _residual + _rlResidual;
    const bool weakensBraking = needsBraking &&
                               _speedCmS * combinedCorrection > 0.0f;
    const bool worsensExcursion = distance > OUTWARD_GUARD_ERROR_CM &&
                                  e * _speedCmS >= 0.0f &&
                                  e * combinedCorrection > 0.0f;
    if (weakensBraking || worsensExcursion) {
        _residual = 0.0f;
        _rlResidual = 0.0f;
        _residualApplied = false;
        _rlResidualApplied = false;
    } else if (peakWeight > 0.0f && _speedCmS * combinedCorrection > 0.0f) {
        // Fade only a net correction that accelerates the outward motion.
        // Keep NN/RL contributions paired and retain inward-helping sums.
        const float correctionWeight = 1.0f - peakWeight;
        _residual *= correctionWeight;
        _rlResidual *= correctionWeight;
        if (correctionWeight <= 0.0f) {
            _residualApplied = false;
            _rlResidualApplied = false;
        }
    }
    // Keep the learned trim and NN input unchanged. Only attenuate the
    // positive bias used for actuator effort while shallow, at any speed.
    // Ramp over 2cm; never remove more than 0.2cm/s^2.
    const float shallowWeight = clampf(-e / 2.0f, 0.0f, 1.0f);
    const float positiveBias = clampf(_bias, 0.0f, 0.2f);
    const float effectiveBias = _bias - positiveBias * shallowWeight;
    // Keep NN and RL separately bounded; do not clamp their sum.
    const float effort = _aDes - effectiveBias + _residual + _rlResidual;
    float duty = (effort > 0.0f) ? effort / G_SINK : effort / G_RISE;

    // Limit the actuator request after all corrections: upward braking must
    // not produce a full-power sinking pulse at/near the target. Release the
    // limit gradually only when shallow enough to require depth recovery.
    // Manual ascent/descent bypass computeDuty and keep their original PWM.
    const float sinkRecoveryWeight = clampf(
        (-e - HOLD_SINK_BAND_CM) / (SINK_RECOVERY_FULL_CM - HOLD_SINK_BAND_CM),
        0.0f, 1.0f);
    const float sinkDutyLimit = HOLD_SINK_DUTY_MAX +
        (1.0f - HOLD_SINK_DUTY_MAX) * sinkRecoveryWeight;
    const bool saturated = fabsf(duty) >= 1.0f || duty > sinkDutyLimit;
    duty = clampf(duty, -1.0f, 1.0f);
    if (duty > sinkDutyLimit) {
        duty = sinkDutyLimit;
    }
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
    if (std::fabs(targetDepthCm - _targetDepthCm) > 0.1f) {
        _student.reset();
        _residual = _rlResidual = 0.0f;
        _residualApplied = _rlResidualApplied = false;
    }
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
    _residualApplied = false;
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
bool DepthController::isResidualApplied() const { return _residualApplied; }
float DepthController::getRLResidualOutput() const { return _rlResidual; }
bool DepthController::isRLResidualApplied() const { return _rlResidualApplied; }
float DepthController::getTotalOutput() const { return _controlOutput; }
bool DepthController::isOutputSaturated() const { return _outputSaturated; }
uint8_t DepthController::getBuoyancyDirection() const { return _buoyancyDirection; }
uint8_t DepthController::getBuoyancyPwm() const { return _buoyancyPwm; }
uint8_t DepthController::getManualDirection() const { return _manualDirection; }

void DepthController::stopBuoyancyOutput() {
    _residualApplied = false;
    _student.reset(); _residual = 0;
    _rlResidual = 0.0f;
    _rlResidualApplied = false;
    _dutyCmd = 0.0f;
    _controlOutput = 0.0f;
    _outputSaturated = false;
    _buoyancyDirection = BUOYANCY_STOP;
    _buoyancyPwm = 0;
}
