/**********************************************************************
 * ForwardControl.cpp — 只发意图，时序在执行器本地（见头文件说明）
 *********************************************************************/

#include "ForwardControl.h"

#include "MotionLink.h"

// 每次改变意图都要同步给 MotionLink —— 它负责每 200ms 把"前进应该在跑"
// 重申一遍（ACT_SYNC）。没有这一步，ACT_START 丢一帧前进就永久失配。
void ForwardControl::start() {
    _running = true;
    if (_link) {
        _link->setForwardIntent(true);
        _link->sendMotion(SUBSYS_FORWARD, ACT_START);
    }
}

void ForwardControl::stop() {
    if (!_running) return;
    _running = false;
    if (_link) {
        _link->setForwardIntent(false);
        _link->sendMotion(SUBSYS_FORWARD, ACT_STOP);
    }
}

void ForwardControl::forceBalance() {
    _running = false;
    if (_link) {
        _link->setForwardIntent(false);
        _link->sendMotion(SUBSYS_FORWARD, ACT_BALANCE);
    }
}

void ForwardControl::emergencyStop() {
    _running = false;
    if (_link) {
        _link->setForwardIntent(false);
        _link->sendMotion(SUBSYS_FORWARD, ACT_HARD_STOP);
    }
}
