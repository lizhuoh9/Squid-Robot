/**********************************************************************
 * ForwardControl.h / LeftTurnControl.h / RightTurnControl.h 的共同说明
 *
 * 2026-09-20 起：**时序不在这里生成**。阀门节拍由 Minima 执行器按它自己的
 * EEPROM 参数在本地跑，这里只负责在合适的时机发"意图"（开始 / 停止 / 平衡），
 * 并保留一份意图状态供界面显示和自动避障判断。
 *
 * 这样做的原因：ESP32 侧的抖动、丢帧、WiFi/SD 卡顿都不会再打乱阀门节拍。
 *********************************************************************/

#ifndef SQUID_FORWARD_CONTROL_H
#define SQUID_FORWARD_CONTROL_H

#include <cstdint>

#include "Board.h"

class MotionLink;

class ForwardControl {
public:
    void begin(MotionLink* link) { _link = link; }

    void start();
    void stop();           // 有序停止：执行器进入普通平衡窗口
    void forceBalance();   // 急停后的全局平衡
    void emergencyStop();  // 立即停，不平衡

    bool isRunning() const { return _running; }
    bool isBusy() const { return _running; }

private:
    MotionLink* _link = nullptr;
    bool _running = false;
};

#endif  // SQUID_FORWARD_CONTROL_H
