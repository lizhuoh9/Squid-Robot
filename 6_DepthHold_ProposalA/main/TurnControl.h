/**********************************************************************
 * TurnControl.h
 *
 * 左/右转：同样只发意图，转向时长与平衡节拍由执行器本地参数决定。
 * _running 只是给界面和自动避障看的近似状态：按执行器的转向时长自行超时清除
 * （时长从执行器回传的参数镜像里取，默认 1000ms）。
 *********************************************************************/

#ifndef SQUID_TURN_CONTROL_H
#define SQUID_TURN_CONTROL_H

#include <cstdint>

#include "Board.h"

class MotionLink;

class TurnControl {
public:
    // left=true 走 ACT_START（左转），false 走 ACT_START_ALT（右转）
    void begin(MotionLink* link, bool left) { _link = link; _left = left; }

    void start();
    void cancel();
    void forceBalance();
    void update(uint32_t nowMs);          // 只做本地状态超时，不产生时序
    void setDurationMs(uint16_t ms) { _durationMs = ms; }

    bool isRunning() const { return _running; }
    bool isBusy() const { return _running; }

private:
    MotionLink* _link = nullptr;
    bool     _left = true;
    bool     _running = false;
    uint32_t _startMs = 0;
    uint16_t _durationMs = 1000;
};

#endif  // SQUID_TURN_CONTROL_H
