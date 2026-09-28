/**********************************************************************
 * DepthController.cpp
 *
 * 预测式定深控制：使用滤波后的深度、速度、加速度。
 *********************************************************************/

#include "DepthController.h"

#include "Calibration.h"

#include <cmath>

#include "Sys.h"

namespace {
constexpr float DEADBAND_CM = 0.20f;
constexpr float MAX_DEPTH_CM = 100.0f;
constexpr float SPEED_LIMIT_CM_S = 0.8f;
constexpr float SYSTEM_TAU = 4.0f;
constexpr float CONTROL_TRIGGER = 8.0f;
constexpr uint8_t PUMP_PWM_MIN = 80;
constexpr uint8_t PUMP_PWM_MAX = 255;
constexpr uint8_t MANUAL_PWM = 255;


inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
inline int clampi(int v, int lo, int hi) {
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
      _errPrev(0.0f),
      _derivPrev(0.0f),
      _controlOutput(0.0f),
      _kpBase(120.0f),
      _kiBase(1.2f),
      _kdBase(150.0f),
      _kp(120.0f),
      _ki(1.2f),
      _kd(150.0f),
      _manualDirection(BUOYANCY_STOP),
      _buoyancyDirection(BUOYANCY_STOP),
      _buoyancyPwm(0),
      _lastControlUpdateMs(0),
      _balancing(false),
      _balanceStartMs(0),
      _balanceEndMs(0) {}

void DepthController::begin() {
    resetAfterCalibration();
}

// 注意：本函数所有分支只改"意图"(_buoyancyDirection/_buoyancyPwm)，
// 最后由 resolveOutputs() 统一展开成阀门位+PWM。
void DepthController::update(bool depthValid,
                             bool depthFresh,
                             float filteredDepthCm,
                             float depthSpeedCmS,
                             float depthAccelCmS2,
                             uint32_t nowMs) {
    updateIntent(depthValid, depthFresh, filteredDepthCm, depthSpeedCmS, depthAccelCmS2, nowMs);
}

void DepthController::updateIntent(bool depthValid,
                             bool depthFresh,
                             float filteredDepthCm,
                             float depthSpeedCmS,
                             float depthAccelCmS2,
                             uint32_t nowMs) {
    if (_balancing) {
        if (nowMs >= _balanceEndMs) {
            _balancing = false;
            stopBuoyancyOutput();
            return;
        }
        // 平衡期间持续发 BALANCE：E/F 单阀每 500ms 交替泄压由 Minima 执行
        // （与前进/转向的平衡完全一致，不再"两阀全开"或半周期切回 STOP）。
        _buoyancyDirection = BUOYANCY_BALANCE;
        _buoyancyPwm       = 0;
        return;
    }

    if (_manualDirection != BUOYANCY_STOP) {
        _controlOutput     = (_manualDirection == cal::BUOY_SINK) ? 100.0f : -100.0f;
        _buoyancyDirection = _manualDirection;
        _buoyancyPwm       = MANUAL_PWM;
        return;
    }

    if (!depthValid) {
        stopBuoyancyOutput();
        return;
    }

    _depthFilt = filteredDepthCm;
    _speedCmS = depthSpeedCmS;
    _accelCmS2 = depthAccelCmS2;

    // 只在有新深度采样时推进 PID：主循环 ~1kHz 但深度 ~12Hz，否则会拿同一份
    // 读数以 100Hz 重复算、微分项在采样边界尖峰。不新鲜就保持上一次输出。
    if (!depthFresh) {
        return;
    }

    if (_lastControlUpdateMs == 0) {
        _lastControlUpdateMs = nowMs;
        _errPrev = _targetDepthCm - _depthFilt;
        stopBuoyancyOutput();
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

    if (!_holdingTarget) {
        _errPrev = _targetDepthCm - _depthFilt;
        _derivPrev = 0.0f;
        stopBuoyancyOutput();
        return;
    }

    if (_targetDepthCm < 0.0f || _depthFilt >= MAX_DEPTH_CM) {
        clearTarget();
        manualStop();
        stopBuoyancyOutput();
        return;
    }

    const float err = _targetDepthCm - _depthFilt;
    const float derr = (err - _errPrev) / dt;
    adaptivePID(err, derr, _speedCmS, dt);

    float u = _kp * err + _ki * _integ + _kd * _derivPrev;
    speedLimiter(u, err);

    const float uFinal = clampf(u, -100.0f, 100.0f);
    const bool isSaturated =
        (uFinal >= 100.0f && err > 0.0f) ||
        (uFinal <= -100.0f && err < 0.0f);

    if (!isSaturated) {
        _integ += err * dt;
    }
    _integ = clampf(_integ, -100.0f, 100.0f);
    _errPrev = err;
    _controlOutput = uFinal;

    if (fabsf(err) <= DEADBAND_CM || fabsf(uFinal) < CONTROL_TRIGGER) {
        stopBuoyancyOutput();
        return;
    }

    applyBuoyancyOutput(uFinal);
}

void DepthController::setTargetDepth(float targetDepthCm) {
    _targetDepthCm = clampf(targetDepthCm, 0.0f, MAX_DEPTH_CM);
    _holdingTarget = true;
    _integ = 0.0f;
    _manualDirection = BUOYANCY_STOP;
}

void DepthController::holdCurrentDepth() {
    _targetDepthCm = _depthFilt;
    _holdingTarget = true;
    _integ = 0.0f;
    _manualDirection = BUOYANCY_STOP;
}

void DepthController::clearTarget() {
    _holdingTarget = false;
    _targetDepthCm = 0.0f;
    _integ = 0.0f;
    _controlOutput = 0.0f;
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
    _holdingTarget = false;
    _targetDepthCm = 0.0f;
    _depthFilt = 0.0f;
    _speedCmS = 0.0f;
    _accelCmS2 = 0.0f;
    _integ = 0.0f;
    _errPrev = 0.0f;
    _derivPrev = 0.0f;
    _controlOutput = 0.0f;
    _manualDirection     = BUOYANCY_STOP;
    _buoyancyDirection   = BUOYANCY_STOP;
    _buoyancyPwm         = 0;
    _lastControlUpdateMs = 0;
    _balancing           = false;
    _balanceEndMs        = 0;
    _balanceStartMs      = 0;
}

float DepthController::getFilteredDepthCm() const { return _depthFilt; }
float DepthController::getSpeedCmS() const { return _speedCmS; }
float DepthController::getAccelerationCmS2() const { return _accelCmS2; }
float DepthController::getTargetDepthCm() const { return _targetDepthCm; }
float DepthController::getControlOutput() const { return _controlOutput; }
uint8_t DepthController::getBuoyancyDirection() const { return _buoyancyDirection; }
uint8_t DepthController::getBuoyancyPwm() const { return _buoyancyPwm; }
uint8_t DepthController::getManualDirection() const { return _manualDirection; }

void DepthController::adaptivePID(float err, float derr, float spd, float dt) {
    const float aErr = fabsf(err);
    const float aRate = fabsf(derr);
    const float aSpd = fabsf(spd);

    if (aErr > 15.0f) {
        _kp = _kpBase * 2.0f;
        _ki = _kiBase * 0.7f;
        _kd = _kdBase * 1.5f;
    } else if (aErr > 5.0f) {
        _kp = _kpBase * 1.2f;
        _ki = _kiBase * 1.0f;
        _kd = _kdBase * 1.2f;
    } else {
        _kp = _kpBase * 1.0f;
        _ki = _kiBase * 1.3f;
        _kd = _kdBase * 1.0f;
    }

    if (aRate > 2.0f) {
        _kd *= 1.3f;
    } else if (aRate < 0.5f) {
        _kd *= 0.8f;
        _ki *= 1.1f;
    }

    if (aSpd > 3.0f) {
        _kd *= 1.2f;
        _kp *= 0.8f;
    } else if (aSpd < 0.5f && aErr < 3.0f) {
        _ki *= 1.3f;
        _kp *= 0.9f;
    }

    if (spd > 0.0f && err > 0.0f) {
        _kp *= 1.7f;
        _kd *= 1.7f;
    }

    _kp = clampf(_kp, _kpBase * 0.3f, _kpBase * 2.5f);
    _ki = clampf(_ki, _kiBase * 0.2f, _kiBase * 2.0f);
    _kd = clampf(_kd, _kdBase * 0.3f, _kdBase * 3.0f);

    const float derivAlpha = clampf(dt * 4.0f, 0.05f, 0.35f);
    _derivPrev += (derr - _derivPrev) * derivAlpha;
}

void DepthController::speedLimiter(float& u, float err) {
    if (fabsf(_speedCmS) > SPEED_LIMIT_CM_S) {
        if ((_speedCmS > 0.0f && u > 0.0f) || (_speedCmS < 0.0f && u < 0.0f)) {
            u *= 0.5f;
        }
    }

    const float kinematicPredict =
        _speedCmS * SYSTEM_TAU +
        0.5f * _accelCmS2 * SYSTEM_TAU * SYSTEM_TAU;

    float originalPredict = fmaxf(fabsf(_speedCmS) * SYSTEM_TAU, 0.5f * fabsf(err));
    if (_speedCmS > 0.0f) {
        originalPredict *= 2.0f;
    }

    float finalPredict = fmaxf(fabsf(kinematicPredict), originalPredict);
    finalPredict = clampf(finalPredict, 2.0f, 25.0f);

    if (fabsf(err) < finalPredict && fabsf(_speedCmS) > SPEED_LIMIT_CM_S) {
        const float accelFactor = 1.0f + clampf(fabsf(_accelCmS2) * 0.5f, 0.0f, 1.0f);

        if (_speedCmS > 0.0f && err > 0.0f) {
            u = -fabsf(u) * 2.0f * accelFactor;
        } else if (_speedCmS < 0.0f && err < 0.0f) {
            u = fabsf(u) * 1.2f;
        } else if (_speedCmS > 0.0f && err < 0.0f) {
            u = -fabsf(u) * 2.5f * accelFactor;
        } else if (_speedCmS < 0.0f && err > 0.0f) {
            u = fabsf(u) * 1.2f;
        }
    }
}

void DepthController::applyBuoyancyOutput(float u) {
    const float magnitude = clampf(fabsf(u), CONTROL_TRIGGER, 100.0f);
    const float span = fmaxf(1.0f, 100.0f - CONTROL_TRIGGER);
    const float normalized = (magnitude - CONTROL_TRIGGER) / span;
    const float pwm = PUMP_PWM_MIN + normalized * (PUMP_PWM_MAX - PUMP_PWM_MIN);

    // err = 目标深度 - 当前深度，u > 0 表示要往更深处去 → 下沉。
    // 原来这里写的是 BUOYANCY_DESCEND，但那个常量物理上是上浮，定深会朝反方向跑飞。
    _buoyancyDirection = u > 0.0f ? cal::BUOY_SINK : cal::BUOY_RISE;
    _buoyancyPwm = static_cast<uint8_t>(
        clampi(static_cast<int>(pwm + 0.5f), PUMP_PWM_MIN, PUMP_PWM_MAX));
}

void DepthController::stopBuoyancyOutput() {
    _controlOutput = 0.0f;
    _buoyancyDirection = BUOYANCY_STOP;
    _buoyancyPwm = 0;
}
