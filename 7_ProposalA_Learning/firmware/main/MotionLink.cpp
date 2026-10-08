/**********************************************************************
 * MotionLink.cpp
 *
 * Minima 执行器/浮沉命令的串口传输（UART1，2Mbps）。
 * UART1 收发都在这里 begin，StatusDisplay 复用同一端口读回传帧。
 *********************************************************************/

#include "MotionLink.h"

#include "Sys.h"
#include "driver/uart.h"

MotionLink::MotionLink()
    : _lastBuoyancyDirection(BUOYANCY_STOP),
      _lastBuoyancyPwm(0) {}

void MotionLink::begin() {
    const uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_TO_MINIMA_PORT, 512, 512, 0, nullptr, 0);
    uart_param_config(UART_TO_MINIMA_PORT, &cfg);
    uart_set_pin(UART_TO_MINIMA_PORT, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void MotionLink::applyBuoyancy(uint8_t dir, uint8_t pwm, bool forceSend) {
    if (!forceSend && dir == _lastBuoyancyDirection && pwm == _lastBuoyancyPwm) {
        return;
    }
    sendBuoyancyIntent(dir, pwm);
    _lastBuoyancyDirection = dir;
    _lastBuoyancyPwm = pwm;
}

void MotionLink::sendMotion(uint8_t subsys, uint8_t action, uint8_t arg, uint8_t repeat) {
    for (uint8_t i = 0; i < repeat; ++i) {
        sendCommand(CMD_MOTION, subsys, action, arg);
    }
}

void MotionLink::setParam(uint8_t id, uint16_t value) {
    sendCommand(CMD_SET_PARAM, id, static_cast<uint8_t>(value & 0xFF),
                static_cast<uint8_t>(value >> 8));
}

void MotionLink::saveParams() { sendCommand(CMD_SAVE_PARAMS); }
void MotionLink::requestParams() { sendCommand(CMD_GET_PARAMS); }

// 浮沉意图 → CMD_MOTION 动作码。
void MotionLink::sendBuoyancyIntent(uint8_t dir, uint8_t pwm) {
    uint8_t action = ACT_STOP;
    if (dir == BUOYANCY_ASCEND)       action = ACT_START;
    else if (dir == BUOYANCY_DESCEND) action = ACT_START_ALT;
    else if (dir == BUOYANCY_BALANCE) action = ACT_BALANCE;
    sendMotion(SUBSYS_BUOYANCY, action, pwm, 1);
}

// 周期性重申当前意图（详细理由见 MotionLink.h）。两条都是幂等的，
// 所以可以无限重发：丢一帧最多 200ms 就自己补回来。
void MotionLink::refresh(uint32_t nowMs) {
    constexpr uint32_t LINK_REFRESH_MS = 200;
    if (nowMs - _lastTxMs < LINK_REFRESH_MS) return;
    sendBuoyancyIntent(_lastBuoyancyDirection, _lastBuoyancyPwm);
    // 前进：只在"应该在跑"时重申。停止不需要重申——ACT_STOP 丢了的话
    // 执行器会继续跑，而重发 ACT_STOP 会把平衡窗口反复重置；那一侧由
    // main.cpp 里的意图/实际核对兜底。
    if (_forwardIntent) {
        sendMotion(SUBSYS_FORWARD, ACT_SYNC, 0, 1);
    }
}

void MotionLink::emergencyStop() {
    sendCommand(CMD_EMERGENCY_STOP);
    _forwardIntent = false;
    _lastBuoyancyDirection = BUOYANCY_STOP;
    _lastBuoyancyPwm = 0;
}

void MotionLink::sendCommand(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2) {
    uint8_t frame[FRAME_LENGTH];
    frame[0] = FRAME_HEADER;
    frame[1] = FRAME_LENGTH;
    frame[2] = cmd;
    frame[3] = data0;
    frame[4] = data1;
    frame[5] = data2;
    frame[6] = calculateCRC8(&frame[2], 4);
    frame[7] = FRAME_TAIL;
    ++_framesSent;
    _lastTxMs = millis();
    uart_write_bytes(UART_TO_MINIMA_PORT, reinterpret_cast<const char*>(frame), FRAME_LENGTH);
}
