/**********************************************************************
 * MotionSeq.cpp — 执行器本地时序状态机（周期全部来自 MotionParams）
 *********************************************************************/

#include "MotionSeq.h"

// ══ 前进 ═══════════════════════════════════════════════════════════
void ForwardSeq::start() {
    _running = true;
    _balancing = false;
    _valvesOpen = true;     // 启动时阀开，第一个节拍动作是关阀建压
    _balValveOpen = false;
    _balAlt = false;
    _phaseMs = 0;           // 由第一次 update() 初始化
}

void ForwardSeq::stop() {
    if (!_running) return;
    _running = false;
    _balancing = true;
    _valvesOpen = false;
    _balValveOpen = false;
    _balAlt = false;
    _balStartMs = 0;
    _balAltMs = g_params[P_FWD_BAL_ALT];
    _balTotalMs = g_params[P_FWD_BAL_TIME];
}

void ForwardSeq::forceBalance() {
    _running = false;
    _balancing = true;
    _valvesOpen = false;
    _balValveOpen = false;
    _balAlt = false;
    _balStartMs = 0;
    _balAltMs = g_params[P_GLOBAL_BAL_ALT];
    _balTotalMs = g_params[P_GLOBAL_BAL_TIME];
}

void ForwardSeq::emergencyStop() {
    _running = false;
    _balancing = false;
    _valvesOpen = false;
    _balValveOpen = false;
    _balAlt = false;
    _phaseMs = 0;
    _balStartMs = 0;
}

void ForwardSeq::update(uint32_t nowMs) {
    if (_running) {
        if (_phaseMs == 0) {
            _phaseMs = nowMs;
        } else if (nowMs - _phaseMs >= g_params[P_FWD_INTERVAL]) {
            _valvesOpen = !_valvesOpen;
            _phaseMs = nowMs;
        }
    }

    if (_balancing) {
        if (_balStartMs == 0) _balStartMs = nowMs;
        const uint32_t elapsed = nowMs - _balStartMs;
        const uint16_t delay = g_params[P_FWD_BAL_DELAY];
        _balValveOpen = elapsed >= delay && elapsed < static_cast<uint32_t>(delay) + _balTotalMs;
        if (_balValveOpen && _balAltMs > 0) {
            _balAlt = ((elapsed - delay) / _balAltMs) % 2 != 0;
        }
        if (elapsed >= static_cast<uint32_t>(delay) + _balTotalMs) {
            _balancing = false;
            _balValveOpen = false;
            _balAlt = false;
        }
    }
}

uint16_t ForwardSeq::mask() const {
    uint16_t m = 0;
    if (_running) {
        m |= ACT_FORWARD_PUMP;
        if (_valvesOpen) m |= ACT_FORWARD_VALVE_A | ACT_FORWARD_VALVE_B;
    }
    if (_balancing && _balValveOpen) {
        m |= _balAlt ? ACT_FORWARD_VALVE_B : ACT_FORWARD_VALVE_A;
    }
    return m;
}

// ══ 转向 ═══════════════════════════════════════════════════════════
void TurnSeq::start(bool left) {
    _running = true;
    _left = left;
    _balancing = false;
    _balValveOpen = false;
    _balAlt = false;
    _startMs = 0;
}

void TurnSeq::cancel() {
    _running = false;
    _balancing = false;
    _balValveOpen = false;
    _balAlt = false;
    _startMs = 0;
}

void TurnSeq::forceBalance() {
    _running = false;
    _balancing = true;
    _balValveOpen = false;
    _balAlt = false;
    _startMs = 0;
    _balStartMs = 0;
    _balAltMs = g_params[P_GLOBAL_BAL_ALT];
    _balTotalMs = g_params[P_GLOBAL_BAL_TIME];
}

void TurnSeq::update(uint32_t nowMs) {
    if (_running && _startMs == 0) {
        _startMs = nowMs;
    }
    if (_running && _startMs != 0 && nowMs - _startMs >= g_params[P_TURN_DURATION]) {
        _running = false;
        _balancing = true;
        _balValveOpen = false;
        _balAlt = false;
        _balStartMs = nowMs;
        _balAltMs = g_params[P_TURN_BAL_ALT];
        _balTotalMs = g_params[P_TURN_BAL_TIME];
    }

    if (_balancing) {
        if (_balStartMs == 0) _balStartMs = nowMs;
        const uint32_t elapsed = nowMs - _balStartMs;
        const uint16_t delay = g_params[P_TURN_BAL_DELAY];
        _balValveOpen = elapsed >= delay && elapsed < static_cast<uint32_t>(delay) + _balTotalMs;
        if (_balValveOpen && _balAltMs > 0) {
            _balAlt = ((elapsed - delay) / _balAltMs) % 2 != 0;
        }
        if (elapsed >= static_cast<uint32_t>(delay) + _balTotalMs) {
            _balancing = false;
            _balValveOpen = false;
            _balAlt = false;
        }
    }
}

uint16_t TurnSeq::mask() const {
    uint16_t m = 0;
    if (_running) {
        m |= ACT_TURN_PUMP;
        if (_left) m |= ACT_TURN_VALVE_C | ACT_TURN_VALVE_D;   // 左转=正循环；右转=阀全关
    }
    if (_balancing && _balValveOpen) {
        m |= _balAlt ? ACT_TURN_VALVE_D : ACT_TURN_VALVE_C;
    }
    return m;
}

// ══ 浮沉 ═══════════════════════════════════════════════════════════
void BuoySeq::setIntent(uint8_t dir, uint8_t pwm, uint32_t nowMs) {
    if (dir == BUOYANCY_BALANCE && _dir != BUOYANCY_BALANCE) {
        _balStartMs = nowMs;   // 平衡相位从进入那一刻起算
    }
    _dir = dir;
    _wantPwm = (dir == BUOYANCY_STOP || dir == BUOYANCY_BALANCE) ? 0 : pwm;
}

void BuoySeq::stop() {
    _dir = BUOYANCY_STOP;
    _wantPwm = 0;
    _valves = 0;
    _pwm = 0;
}

void BuoySeq::update(uint32_t nowMs) {
    uint8_t valves = 0;
    uint8_t pwm = 0;

    switch (_dir) {
        case BUOYANCY_ASCEND:
            valves = 0;                              // E/F 全关
            pwm = _wantPwm;
            break;
        case BUOYANCY_DESCEND:
            valves = BUOY_VALVE_E | BUOY_VALVE_F;    // E/F 全开
            pwm = _wantPwm;
            break;
        case BUOYANCY_BALANCE: {
            const uint16_t alt = g_params[P_BUOY_BAL_ALT];
            const bool phaseF = alt ? (((nowMs - _balStartMs) / alt) % 2 != 0) : false;
            valves = phaseF ? BUOY_VALVE_F : BUOY_VALVE_E;
            pwm = 0;
            break;
        }
        default:
            valves = 0;
            pwm = 0;
            break;
    }

    if (valves != _valves) {
        _valves = valves;
        _valveChangeMs = nowMs;
    }
    // 换向保护：刚动过阀门先不给泵加力，避免冲击。
    if (pwm > 0 && nowMs - _valveChangeMs < g_params[P_VALVE_GUARD]) {
        pwm = 0;
    }
    _pwm = pwm;
}
