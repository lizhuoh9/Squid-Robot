/**********************************************************************
 * TurnControl.cpp — 只发意图（见头文件说明）
 *********************************************************************/

#include "TurnControl.h"

#include "MotionLink.h"
#include "Sys.h"

void TurnControl::start() {
    _running = true;
    _startMs = millis();
    if (_link) _link->sendMotion(SUBSYS_TURN, _left ? ACT_START : ACT_START_ALT);
}

void TurnControl::cancel() {
    _running = false;
    if (_link) _link->sendMotion(SUBSYS_TURN, ACT_STOP);
}

void TurnControl::forceBalance() {
    _running = false;
    if (_link) _link->sendMotion(SUBSYS_TURN, ACT_BALANCE);
}

void TurnControl::update(uint32_t nowMs) {
    if (_running && nowMs - _startMs >= _durationMs) {
        _running = false;   // 执行器那边此刻已转入平衡窗口
    }
}
