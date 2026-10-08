/**********************************************************************
 * MotionLink.h
 *
 * ESP32 运动/控制逻辑到 Minima 执行端的串口链路（UART1）。
 *********************************************************************/

#ifndef SQUID_MOTION_LINK_H
#define SQUID_MOTION_LINK_H

#include <cstdint>

#include "Board.h"

class MotionLink {
public:
    MotionLink();

    void begin();
    // 浮沉意图：dir = BUOYANCY_STOP/ASCEND/DESCEND/BALANCE，pwm = 泵力度。
    void applyBuoyancy(uint8_t dir, uint8_t pwm, bool forceSend = false);
    void emergencyStop();

    // 前进意图：ForwardControl 一改状态就告诉这里，refresh() 负责周期重申。
    void setForwardIntent(bool running) { _forwardIntent = running; }

    // 每 LINK_REFRESH_MS 把当前意图重申一遍（浮沉 + 前进）。
    //
    // 为什么必须有：2026-09-20 的重构把下发从"每次变化发完整掩码"（状态型，
    // 丢帧下一帧自动纠正）改成了"按键那一刻发一次 ACT_START"（事件型）。
    // 事件型丢一帧就永久失配：ESP32 以为在前进、执行器停着。当时只给浮沉加了
    // 重申，前进没加 —— 实测日志里 6 次 w 有 2 次执行器根本没动。
    //
    // 两者都是幂等的：浮沉重发同一意图不产生动作；前进走 ACT_SYNC，执行器
    // "没在跑才 start()"，已在跑就不碰节拍相位。转向是一次性 ~1 秒动作，
    // 重申反而会把它无限重启，所以不重申。
    void refresh(uint32_t nowMs);

    // 意图（执行器本地按自己的参数生成时序）。状态变化类命令连发 3 次防丢帧：
    // 执行器对"开始前进"不是幂等的，丢了就不动，多发一次代价很小。
    void sendMotion(uint8_t subsys, uint8_t action, uint8_t arg = 0, uint8_t repeat = 3);
    void setParam(uint8_t id, uint16_t value);
    void saveParams();
    void requestParams();
    uint32_t getFramesSent() const { return _framesSent; }

private:
    uint32_t _framesSent = 0;
    uint32_t _lastTxMs = 0;
    uint8_t _lastBuoyancyDirection;
    uint8_t _lastBuoyancyPwm;
    bool    _forwardIntent = false;

    void sendBuoyancyIntent(uint8_t dir, uint8_t pwm);
    void sendCommand(uint8_t cmd, uint8_t data0 = 0, uint8_t data1 = 0, uint8_t data2 = 0);
};

#endif  // SQUID_MOTION_LINK_H
