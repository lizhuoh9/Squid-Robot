/**********************************************************************
 * Protocol.h
 *
 * 这个文件定义 Minima 侧使用的固定长度控制帧协议。
 *
 * 分工（2026-09-20 起）：ESP32 只下发"意图"和"参数"，执行器本地按参数生成时序。
 *   CMD_MOTION      data0=子系统 data1=动作 data2=参数(浮沉 PWM)
 *   CMD_SET_PARAM   data0=参数号 data1=低字节 data2=高字节（立即生效）
 *   CMD_SAVE_PARAMS 把当前参数写 EEPROM（断电保留）
 *   CMD_GET_PARAMS  把参数打到执行器 USB 串口
 *   CMD_EMERGENCY_STOP 立即停全部输出
 *********************************************************************/

#ifndef MINIMA_PROTOCOL_H
#define MINIMA_PROTOCOL_H

#include <Arduino.h>

#define UART_BAUD_RATE        2000000

#define FRAME_HEADER          0xAA
#define FRAME_TAIL            0x55
#define FRAME_LENGTH          8

// 0x01 / 0x04 曾是"直接写掩码 / 直接写阀门位"的调试通道，2026-09-21 已删除：
//   1) 新架构下 ESP32 从不发它们；
//   2) 它们会把 gDirectMask/gDirectBuoy 永久置位，悄悄停掉本地状态机；
//   3) CMD_SET_BUOYANCY 的 data0 语义在重构中从"方向"变成了"阀门位"，
//      旧固件发过来会做出完全不同的动作。留着只会坑人，收到一律报未知命令。
#define CMD_EMERGENCY_STOP    0x02
#define CMD_MOTION            0x05
#define CMD_SET_PARAM         0x06
#define CMD_SAVE_PARAMS       0x07
#define CMD_GET_PARAMS        0x08

// CMD_MOTION 的 data0：子系统
#define SUBSYS_FORWARD        0x00
#define SUBSYS_TURN           0x01
#define SUBSYS_BUOYANCY       0x02

// CMD_MOTION 的 data1：动作
#define ACT_STOP              0x00   // 前进=有序停止(带平衡) 转向=取消 浮沉=停
#define ACT_START             0x01   // 前进=开始 转向=左转 浮沉=上浮
#define ACT_START_ALT         0x02   // 转向=右转 浮沉=下沉
#define ACT_BALANCE           0x03   // 全局平衡（急停后）/浮沉平衡
#define ACT_HARD_STOP         0x04   // 立即清空，不做平衡
// 状态重申：ESP32 每 200ms 把"前进应该在跑"重发一遍，幂等。
// 已经在跑就什么都不做（不碰节拍相位），没在跑才 start()。
// 有了它，ACT_START 丢一帧也能在 200ms 内自动补上——这是把"事件型"协议
// 退回"状态型"的关键，详见 Minima.ino 顶部说明。
#define ACT_SYNC              0x05

#define STATUS_MOTION         0x81
#define STATUS_HEARTBEAT      0x82
#define STATUS_PARAM          0x83   // data0=参数号 data1=低字节 data2=高字节

// 前进子系统执行器位（泵 + 阀 a/b）。
constexpr uint16_t ACT_FORWARD_PUMP    = 0x0001;
constexpr uint16_t ACT_FORWARD_VALVE_A = 0x0002;
constexpr uint16_t ACT_FORWARD_VALVE_B = 0x0004;

// 转向子系统执行器位（泵 + 阀 c/d）。
constexpr uint16_t ACT_TURN_PUMP       = 0x0008;
constexpr uint16_t ACT_TURN_VALVE_C    = 0x0010;
constexpr uint16_t ACT_TURN_VALVE_D    = 0x0020;

// 浮力子系统执行器位（泵 + 阀 e/f）。
constexpr uint16_t ACT_BUOYANCY_PUMP   = 0x0040;
constexpr uint16_t ACT_BUOYANCY_VALVE_E= 0x0080;
constexpr uint16_t ACT_BUOYANCY_VALVE_F= 0x0100;

// 分组掩码，便于快速判断当前哪个子系统被激活。
constexpr uint16_t ACT_FORWARD_GROUP =
    ACT_FORWARD_PUMP | ACT_FORWARD_VALVE_A | ACT_FORWARD_VALVE_B;
constexpr uint16_t ACT_TURN_GROUP =
    ACT_TURN_PUMP | ACT_TURN_VALVE_C | ACT_TURN_VALVE_D;
constexpr uint16_t ACT_BUOYANCY_GROUP =
    ACT_BUOYANCY_PUMP | ACT_BUOYANCY_VALVE_E | ACT_BUOYANCY_VALVE_F;

constexpr uint8_t BUOYANCY_STOP    = 0;
constexpr uint8_t BUOYANCY_ASCEND  = 1;
constexpr uint8_t BUOYANCY_DESCEND = 2;
constexpr uint8_t BUOYANCY_BALANCE = 3; // 气压平衡：E/F 按参数交替单开，泵关闭

// 浮沉阀门位：BuoySeq 合成、MotionController 照着写引脚（不再经协议下发）
constexpr uint8_t BUOY_VALVE_E = 0x01;
constexpr uint8_t BUOY_VALVE_F = 0x02;

struct ProtocolFrame {
    uint8_t cmd;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
};

uint8_t calculateCRC8(const uint8_t* data, uint8_t len);
void sendFrame(Stream& port, uint8_t cmd, uint8_t data0 = 0, uint8_t data1 = 0, uint8_t data2 = 0);

class ProtocolReceiver {
public:
    ProtocolReceiver();
    bool poll(Stream& port, ProtocolFrame& frame);
    uint16_t badFrames = 0;   // 诊断：长度/帧尾/CRC 错误被丢弃的帧数（累计）

private:
    uint8_t rxBuffer[FRAME_LENGTH];
    uint8_t rxIndex;
};

#endif // MINIMA_PROTOCOL_H
