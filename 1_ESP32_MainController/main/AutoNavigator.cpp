/**********************************************************************
 * AutoNavigator.cpp
 *
 * 基于超声波数据的自动寻路。自动模式下 ESP32 协调前进和转向，
 * Minima 只执行输出掩码。
 *********************************************************************/

#include "AutoNavigator.h"

namespace {
// 阈值与演示模式的距离线保持一致（窄缸，2026-09-21 按实测速度定）：
// 前方 12cm 停下转向，侧面 10cm 算贴壁，两侧差 5cm 才算一边明显更空。
// 注意 q 模式没有惯性预判，判的是当前距离，所以实际比演示模式动手更晚。
constexpr float FRONT_BLOCK_CM = 12.0f;
constexpr float SIDE_NEAR_CM = 10.0f;
constexpr float SIDE_PREFERENCE_CM = 5.0f;
constexpr uint32_t TURN_LOCK_MS = 1400;
}  // namespace

AutoNavigator::AutoNavigator()
    : _forwardControl(nullptr),
      _leftTurnControl(nullptr),
      _rightTurnControl(nullptr),
      _enabled(false),
      _decisionLockUntilMs(0) {}

void AutoNavigator::begin(ForwardControl* forwardControl, TurnControl* leftTurn, TurnControl* rightTurn) {
    _forwardControl = forwardControl;
    _leftTurnControl = leftTurn;
    _rightTurnControl = rightTurn;
}

void AutoNavigator::setEnabled(bool enabled) {
    _enabled = enabled;
    if (!enabled && _forwardControl) {
        _forwardControl->emergencyStop();
    }
    _decisionLockUntilMs = 0;
}

bool AutoNavigator::isEnabled() const {
    return _enabled;
}

void AutoNavigator::update(const UltrasonicManager& ultrasonicManager, uint32_t nowMs) {
    if (!_enabled || !_forwardControl || !_leftTurnControl || !_rightTurnControl) {
        return;
    }

    if (nowMs < _decisionLockUntilMs) {
        return;
    }

    const bool frontValid = ultrasonicManager.isValid(SENSOR_FRONT);
    const bool leftValid = ultrasonicManager.isValid(SENSOR_LEFT);
    const bool rightValid = ultrasonicManager.isValid(SENSOR_RIGHT);

    const float frontCm = frontValid ? ultrasonicManager.getDistance(SENSOR_FRONT) / 10.0f : 0.0f;
    const float leftCm = leftValid ? ultrasonicManager.getDistance(SENSOR_LEFT) / 10.0f : 0.0f;
    const float rightCm = rightValid ? ultrasonicManager.getDistance(SENSOR_RIGHT) / 10.0f : 0.0f;

    if (!frontValid) {
        _forwardControl->stop();
        return;
    }

    if (frontCm < FRONT_BLOCK_CM) {
        _forwardControl->stop();

        if (leftValid && (!rightValid || leftCm >= rightCm + SIDE_PREFERENCE_CM)) {
            _rightTurnControl->cancel();
            _leftTurnControl->start();
        } else {
            _leftTurnControl->cancel();
            _rightTurnControl->start();
        }

        _decisionLockUntilMs = nowMs + TURN_LOCK_MS;
        return;
    }

    if (leftValid && leftCm < SIDE_NEAR_CM && (!rightValid || rightCm > leftCm + SIDE_PREFERENCE_CM)) {
        _forwardControl->stop();
        _leftTurnControl->cancel();
        _rightTurnControl->start();
        _decisionLockUntilMs = nowMs + TURN_LOCK_MS;
        return;
    }

    if (rightValid && rightCm < SIDE_NEAR_CM && (!leftValid || leftCm > rightCm + SIDE_PREFERENCE_CM)) {
        _forwardControl->stop();
        _rightTurnControl->cancel();
        _leftTurnControl->start();
        _decisionLockUntilMs = nowMs + TURN_LOCK_MS;
        return;
    }

    // 只有当前没在跑才 start()。原来这里无条件调，主循环约 1ms 一轮，
    // 等于每毫秒给执行器发一次 ACT_START，而 ForwardSeq::start() 里有
    // _phaseMs = 0（重置节拍相位）—— update() 刚把它设成 nowMs，下一毫秒
    // 又被清零，永远到不了 700ms 的切换点：阀 A/B 恒开、泵常转，
    // 气腔只充不泄。旧 Arduino 版的 _phaseStartMs = 0 也是同样的坑。
    // 这一行就是避障模式从来没真正游起来过的原因。
    if (!_forwardControl->isRunning()) {
        _forwardControl->start();
    }
}
