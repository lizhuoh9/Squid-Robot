/**********************************************************************
 * MotionControl.h
 *
 * Minima 底层执行器驱动（前进/转向/浮沉）。
 *
 * 前进/转向：掩码驱动，applyMask() 收到帧后立即写引脚。
 * 浮沉：     软件 PWM 驱动，update() 每帧计算占空比并控制气泵。
 *            阀门方向切换有 200ms 防抖延迟，防止机械冲击。
 *********************************************************************/

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <Arduino.h>
#include "PinDefinitions.h"
#include "Protocol.h"

class MotionController {
public:
    MotionController();

    void begin();

    // 接收执行器掩码并立即写前进/转向引脚。
    void applyMask(uint16_t mask);

    // 设置浮沉请求；实际输出由 update() 每帧执行。
    // 浮沉：valveBits bit0=阀E bit1=阀F（1=通电），pwm=泵占空比 0~255。
    // 所有时序（平衡交替、换向保护、上浮/下沉的阀门组合）都由 ESP32 决定，
    // 执行器只负责照着写引脚 + 生成软件 PWM。
    void applyBuoyancy(uint8_t valveBits, uint8_t pwm);

    // 每帧调用：处理浮沉软件 PWM 输出。
    void update();

    // 全局急停：清空所有状态并立即关闭全部引脚。
    void emergencyStopAll();

    uint16_t getMask() const;
    bool isForwardActive() const;
    bool isTurnActive() const;
    bool isBuoyancyActive() const;
    bool isAnyActive() const;
    uint8_t getBuoyancyValves() const;
    uint8_t getBuoyancyPwm() const;

    void printStatus();

private:
    uint16_t _mask;
    uint8_t _buoyancyValves;   // bit0=E bit1=F
    uint8_t _buoyancyPwm;

    void writeOutputs();
    void writeBuoyancyOutputs(uint32_t nowMs);
};

#endif // MOTION_CONTROL_H
