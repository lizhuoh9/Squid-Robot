/**********************************************************************
 * MotionSeq.h
 *
 * 执行器本地时序状态机：前进、转向、浮沉。
 *
 * ESP32 只下发意图（开始前进 / 停止 / 左转 / 右转 / 浮沉方向+力度 / 急停平衡），
 * 这里按 MotionParams 里的周期在本地生成阀门节拍。链路抖动、丢帧、上位机卡顿
 * 都不会影响节拍——上位机只要定期刷新意图即可。
 *********************************************************************/

#ifndef MINIMA_MOTION_SEQ_H
#define MINIMA_MOTION_SEQ_H

#include <Arduino.h>

#include "MotionParams.h"
#include "Protocol.h"

// ── 前进：泵常开，阀 A/B 同时开关交替；停止后两阀轮流单开泄压归中 ──
class ForwardSeq {
public:
    void start();
    void stop();           // 有序停止：进入普通平衡窗口
    void forceBalance();   // 急停后的全局平衡（更长、更慢）
    void emergencyStop();  // 立即清空，不平衡
    void update(uint32_t nowMs);
    uint16_t mask() const;
    bool running() const { return _running; }
    bool busy() const { return _running || _balancing; }

private:
    bool     _running = false;
    bool     _balancing = false;
    bool     _valvesOpen = false;
    bool     _balValveOpen = false;
    bool     _balAlt = false;
    uint32_t _phaseMs = 0;
    uint32_t _balStartMs = 0;
    uint16_t _balAltMs = 0;
    uint16_t _balTotalMs = 0;
};

// ── 转向：左转 = 泵开 + 阀 C/D 同开；右转 = 泵开 + 阀全关；之后轮流泄压 ──
class TurnSeq {
public:
    void start(bool left);
    void cancel();
    void forceBalance();
    void update(uint32_t nowMs);
    uint16_t mask() const;
    bool running() const { return _running; }
    bool busy() const { return _running || _balancing; }

private:
    bool     _running = false;
    bool     _left = true;
    bool     _balancing = false;
    bool     _balValveOpen = false;
    bool     _balAlt = false;
    uint32_t _startMs = 0;
    uint32_t _balStartMs = 0;
    uint16_t _balAltMs = 0;
    uint16_t _balTotalMs = 0;
};

// ── 浮沉：上浮=E/F 全关+泵；下沉=E/F 全开+泵；平衡=E/F 交替单开、泵停 ──
class BuoySeq {
public:
    void setIntent(uint8_t dir, uint8_t pwm, uint32_t nowMs);
    void update(uint32_t nowMs);
    void stop();
    uint8_t valves() const { return _valves; }
    uint8_t pwm() const { return _pwm; }
    uint8_t intent() const { return _dir; }
    bool active() const { return _pwm > 0 || _valves != 0; }

private:
    uint8_t  _dir = BUOYANCY_STOP;
    uint8_t  _wantPwm = 0;
    uint8_t  _valves = 0;
    uint8_t  _pwm = 0;
    uint32_t _balStartMs = 0;
    uint32_t _valveChangeMs = 0;
};

#endif  // MINIMA_MOTION_SEQ_H
