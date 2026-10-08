/**********************************************************************
 * Board.h
 *
 * 引脚映射、通信协议、执行器位掩码——从原 Arduino 固件 Protocol.h 迁移。
 * 原文件里用 `#define Serial1/Serial2` 指代 UART 外设，这里直接给出
 * ESP-IDF 的 uart_port_t 与 GPIO 编号。
 *********************************************************************/

#ifndef SQUID_BOARD_H
#define SQUID_BOARD_H

#include <cstddef>
#include <cstdint>
#include "driver/gpio.h"
#include "driver/uart.h"

// ── ESP32 → Minima 有线串口（原 Serial1）────────────────────────────
constexpr uart_port_t UART_TO_MINIMA_PORT = UART_NUM_1;
constexpr int         UART_BAUD_RATE      = 2000000;
constexpr gpio_num_t  UART_TX_PIN         = GPIO_NUM_17;
constexpr gpio_num_t  UART_RX_PIN         = GPIO_NUM_18;

// ── HC-12 无线串口（原 Serial2）─────────────────────────────────────
//   Arduino: HC12_SERIAL.begin(baud, 8N1, rx=IO1, tx=IO2)
constexpr uart_port_t HC12_UART_PORT = UART_NUM_2;
constexpr int         HC12_BAUD_RATE = 9600;
constexpr int         HC12_TX_BUF_SIZE = 1024;  // HC-12 发送缓冲（满则丢弃，不阻塞）
                                                // 9600 波特下 1KB ≈ 1.1s 积压上限：
                                                // 再大只会让命令回复排在调试文字后面。
constexpr gpio_num_t  HC12_TX_PIN    = GPIO_NUM_2;
constexpr gpio_num_t  HC12_RX_PIN    = GPIO_NUM_1;
constexpr gpio_num_t  HC12_SET_PIN   = GPIO_NUM_47;

// ── 控制台串口（原 Arduino Serial，USB/UART0）───────────────────────
//   仅由 IDF 的 console 组件驱动（stdout/stdin），此处不重复定义外设。

// ── CH9434A SPI 接线（SPI2/FSPI）────────────────────────────────────
constexpr gpio_num_t SPI_MOSI    = GPIO_NUM_11;
constexpr gpio_num_t SPI_MISO    = GPIO_NUM_13;
constexpr gpio_num_t SPI_SCK     = GPIO_NUM_12;
constexpr gpio_num_t SPI_CS      = GPIO_NUM_10;
constexpr gpio_num_t CH9434A_INT = GPIO_NUM_9;

// ── SD 卡 SPI 接线（SPI3/HSPI，独立总线）────────────────────────────
constexpr gpio_num_t SD_SPI_MOSI = GPIO_NUM_6;
constexpr gpio_num_t SD_SPI_MISO = GPIO_NUM_7;
constexpr gpio_num_t SD_SPI_SCK  = GPIO_NUM_8;
constexpr gpio_num_t SD_SPI_CS   = GPIO_NUM_14;

// ── 电池电压 ADC（分压 R1=390k(实测350k)/R2=100k，校准比 5.507）────
//   IO3 → ESP32-S3 ADC1_CH2。
constexpr gpio_num_t BATT_ADC_GPIO      = GPIO_NUM_3;
constexpr float      BATT_DIVIDER_RATIO = 5.507f;
constexpr float      BATT_ADC_REF_V     = 3.114f;
constexpr float      BATT_ADC_MAX       = 4095.0f;
constexpr uint8_t    BATT_SAMPLES       = 16;
// 电池电压取 3 秒窗口最大值（泵阀工作时会压降）：250ms × 12 = 3s。
constexpr uint8_t    BATT_WINDOW        = 12;

// ── MS5837 深度传感器（I2C，实物 SDA/SCL 与丝印相反，软件交换）──────
constexpr uint8_t    DEPTH_I2C_ADDRESS = 0x76;
constexpr gpio_num_t DEPTH_I2C_SDA     = GPIO_NUM_5;   // 实物接到 IO5
constexpr gpio_num_t DEPTH_I2C_SCL     = GPIO_NUM_4;   // 实物接到 IO4
constexpr int        DEPTH_I2C_FREQ    = 400000;

// ── CH9434A 的 UART 通道分配 ────────────────────────────────────────
constexpr uint8_t ULTRASONIC_FRONT_UART = 1;
constexpr uint8_t ULTRASONIC_LEFT_UART  = 2;
constexpr uint8_t ULTRASONIC_RIGHT_UART = 0;
constexpr uint8_t IMU_UART              = 3;

// ── IMU (EBIMU-9DOFV6) 串口参数 ─────────────────────────────────────
constexpr int IMU_BAUDRATE = 115200;

// ── 固定长度协议帧 ──────────────────────────────────────────────────
constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t FRAME_TAIL   = 0x55;
constexpr uint8_t FRAME_LENGTH = 8;

// ── 控制命令 ────────────────────────────────────────────────────────
// 0x01/0x04（直写掩码 / 直写阀门位）已于 2026-09-21 从两端删除：新架构不发它们，
// 而且它们会把执行器的 gDirectMask 永久置位、悄悄停掉本地状态机。
constexpr uint8_t CMD_EMERGENCY_STOP = 0x02;
constexpr uint8_t CMD_MOTION         = 0x05;   // 意图：子系统+动作+参数
constexpr uint8_t CMD_SET_PARAM      = 0x06;   // 参数号 + 16 位值
constexpr uint8_t CMD_SAVE_PARAMS    = 0x07;   // 写执行器 EEPROM
constexpr uint8_t CMD_GET_PARAMS     = 0x08;   // 请求回传全部参数

constexpr uint8_t SUBSYS_FORWARD  = 0x00;
constexpr uint8_t SUBSYS_TURN     = 0x01;
constexpr uint8_t SUBSYS_BUOYANCY = 0x02;

constexpr uint8_t ACT_STOP      = 0x00;   // 前进=有序停止 转向=取消 浮沉=停
constexpr uint8_t ACT_START     = 0x01;   // 前进=开始 转向=左转 浮沉=上浮
constexpr uint8_t ACT_START_ALT = 0x02;   // 转向=右转 浮沉=下沉
constexpr uint8_t ACT_BALANCE   = 0x03;   // 全局平衡 / 浮沉平衡
constexpr uint8_t ACT_HARD_STOP = 0x04;   // 立即清空，不平衡
// 状态重申（幂等）：执行器收到后“没在跑才 start()”，已在跑就不动。
// MotionLink::refresh() 每 200ms 发一次，把“事件型”协议退回“状态型”：
// ACT_START 丢一帧也能在 200ms 内自动补上。详见 MotionLink.h。
constexpr uint8_t ACT_SYNC      = 0x05;

// 执行器本地参数表（存在它的 EEPROM 里；ESP32 只负责下发修改与显示）。
constexpr uint8_t P_FWD_INTERVAL     = 0;
constexpr uint8_t P_FWD_BAL_DELAY    = 1;
constexpr uint8_t P_FWD_BAL_TIME     = 2;
constexpr uint8_t P_FWD_BAL_ALT      = 3;
constexpr uint8_t P_TURN_DURATION    = 4;
constexpr uint8_t P_TURN_BAL_DELAY   = 5;
constexpr uint8_t P_TURN_BAL_TIME    = 6;
constexpr uint8_t P_TURN_BAL_ALT     = 7;
constexpr uint8_t P_GLOBAL_BAL_TIME  = 8;
constexpr uint8_t P_GLOBAL_BAL_ALT   = 9;
constexpr uint8_t P_BUOY_BAL_ALT     = 10;
constexpr uint8_t P_VALVE_GUARD      = 11;
constexpr uint8_t P_PWM_PERIOD_100US = 12;
constexpr uint8_t P_LINK_TIMEOUT     = 13;
constexpr uint8_t P_COUNT            = 14;

constexpr const char* PARAM_NAMES[P_COUNT] = {
    "fwd_interval", "fwd_bal_delay", "fwd_bal_time", "fwd_bal_alt",
    "turn_duration", "turn_bal_delay", "turn_bal_time", "turn_bal_alt",
    "global_bal_time", "global_bal_alt", "buoy_bal_alt", "valve_guard",
    "pwm_period_100us", "link_timeout"
};

// ── Minima 回传状态帧命令号 ─────────────────────────────────────────
constexpr uint8_t STATUS_PARAM     = 0x83;   // data0=参数号 data1/2=值
constexpr uint8_t STATUS_MOTION    = 0x81;
constexpr uint8_t STATUS_HEARTBEAT = 0x82;

// ── 传感器数量与串口参数 ────────────────────────────────────────────
constexpr uint8_t NUM_ULTRASONIC      = 3;
constexpr int     ULTRASONIC_BAUDRATE = 115200;
constexpr uint32_t ULTRASONIC_TIMEOUT = 30;

// ── 超声波逻辑编号 ──────────────────────────────────────────────────
constexpr uint8_t SENSOR_FRONT = 0;
constexpr uint8_t SENSOR_LEFT  = 1;
constexpr uint8_t SENSOR_RIGHT = 2;

// ── 前进子系统执行器位（泵 + 阀 a/b）────────────────────────────────
constexpr uint16_t ACT_FORWARD_PUMP    = 0x0001;
constexpr uint16_t ACT_FORWARD_VALVE_A = 0x0002;
constexpr uint16_t ACT_FORWARD_VALVE_B = 0x0004;

// ── 转向子系统执行器位（泵 + 阀 c/d）────────────────────────────────
constexpr uint16_t ACT_TURN_PUMP    = 0x0008;
constexpr uint16_t ACT_TURN_VALVE_C = 0x0010;
constexpr uint16_t ACT_TURN_VALVE_D = 0x0020;

// ── 浮力子系统执行器位（泵 + 阀 e/f）────────────────────────────────
constexpr uint16_t ACT_BUOYANCY_PUMP    = 0x0040;
constexpr uint16_t ACT_BUOYANCY_VALVE_E = 0x0080;
constexpr uint16_t ACT_BUOYANCY_VALVE_F = 0x0100;

// ── 分组掩码 ────────────────────────────────────────────────────────
constexpr uint16_t ACT_FORWARD_GROUP =
    ACT_FORWARD_PUMP | ACT_FORWARD_VALVE_A | ACT_FORWARD_VALVE_B;
constexpr uint16_t ACT_TURN_GROUP =
    ACT_TURN_PUMP | ACT_TURN_VALVE_C | ACT_TURN_VALVE_D;
constexpr uint16_t ACT_BUOYANCY_GROUP =
    ACT_BUOYANCY_PUMP | ACT_BUOYANCY_VALVE_E | ACT_BUOYANCY_VALVE_F;

// ── 浮沉方向 ────────────────────────────────────────────────────────
// CMD_SET_BUOYANCY 的 data0 = 浮沉阀门位（执行器照着写引脚），data1 = 泵 PWM。
// 所有时序（平衡交替、换向保护、上浮/下沉的阀门组合）都在 ESP32 这边算好。
constexpr uint8_t BUOY_VALVE_E = 0x01;
constexpr uint8_t BUOY_VALVE_F = 0x02;

// 浮沉意图（仅 ESP32 内部使用：日志、面板显示、自动逻辑）。
constexpr uint8_t BUOYANCY_STOP    = 0;
constexpr uint8_t BUOYANCY_ASCEND  = 1;
constexpr uint8_t BUOYANCY_DESCEND = 2;
constexpr uint8_t BUOYANCY_BALANCE = 3;  // 气压平衡：E/F 每 500ms 交替单开，泵关闭

// ── CRC8（ESP32 与 Minima 两侧保持一致，多项式 0x07）───────────────
inline uint8_t calculateCRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

#endif  // SQUID_BOARD_H
