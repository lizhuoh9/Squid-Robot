/**********************************************************************
 * Minima.ino — Squid 执行器（Arduino UNO R4 Minima）
 *
 * 固件已冻结：日常开发只烧 ESP32。时序参数存在本地 EEPROM，ESP32 可随时下发
 * 修改并保存，不需要重烧这块板。
 *
 * 它做三件事：
 *   1. 按 ESP32 的"意图"在本地生成前进/转向/浮沉的阀门节拍（周期来自参数表）；
 *   2. 把节拍写到真实引脚，并为浮力泵生成软件 PWM；
 *   3. 回传运动状态、实际掩码、坏帧计数与最长 loop，供 ESP32 核对。
 *
 * 2026-09-21 修掉的两件事（重构后按 w 时灵时不灵、前进和浮沉无法共存）：
 *   - 加 ACT_SYNC 状态重申。重构把下发从"每次变化发完整掩码"（状态型，丢帧
 *     下一帧自动纠正）改成了"按键那一刻发一次 ACT_START"（事件型），而事件型
 *     丢一帧就永久失配：ESP32 以为在前进、执行器停着。浮沉因为有 200ms 重申
 *     所以能自愈，前进没有。现在 ESP32 每 200ms 也重申一次前进意图，幂等。
 *   - 删掉链路失效保护（checkLinkTimeout / gLinkLost）。它一超时就
 *     forwardSeq.emergencyStop()，把"意图"连同输出一起销毁，而 ESP32 收不到
 *     任何通知 —— 浮沉靠重申回来了，前进再也回不来。ESP32 和本板是 PCB 走线
 *     直连，不存在"失联"，这层保护弊大于利。P_LINK_TIMEOUT 参数槽位保留但已
 *     不再使用（删掉会让 EEPROM 里已存的参数错位）。
 *
 * 与 ESP32 的串口：Serial1，2Mbps，固定 8 字节帧（见 Protocol.h）。
 *********************************************************************/

// 固件版本：执行器逻辑已冻结，日常只烧 ESP32。改这里时记得同步 Minima/README.md。
#define MINIMA_FW_VERSION "Minima-Executor v2.1 (ACT_SYNC, no link-timeout, 2026-09-21)"

#include <Arduino.h>

#include "MotionControl.h"
#include "MotionParams.h"
#include "MotionSeq.h"
#include "Protocol.h"

#define UART_FROM_ESP32 Serial1

MotionController motion;
ProtocolReceiver protocolReceiver;

ForwardSeq forwardSeq;
TurnSeq    turnSeq;
BuoySeq    buoySeq;

unsigned long tStatus = 0;
unsigned long tHeartbeat = 0;
const unsigned long STATUS_INTERVAL = 500;
const unsigned long HEARTBEAT_INTERVAL = 1000;
uint32_t gLoopMaxUs = 0;            // 诊断：心跳周期内最长 loop

static void sendStatusToESP32(uint8_t cmd, uint8_t data0 = 0, uint8_t data1 = 0, uint8_t data2 = 0) {
    sendFrame(UART_FROM_ESP32, cmd, data0, data1, data2);
}

static uint8_t getMotionStatus() {
    uint8_t status = 0;
    if (motion.isForwardActive()) status |= 0x01;
    if (motion.isTurnActive())    status |= 0x02;
    if (motion.isBuoyancyActive())status |= 0x04;
    return status;
}

// ── 意图命令 ───────────────────────────────────────────────────────
static void handleMotion(uint8_t subsys, uint8_t action, uint8_t arg) {
    switch (subsys) {
        case SUBSYS_FORWARD:
            if (action == ACT_START)          forwardSeq.start();
            else if (action == ACT_STOP)      forwardSeq.stop();
            else if (action == ACT_BALANCE)   forwardSeq.forceBalance();
            else if (action == ACT_HARD_STOP) forwardSeq.emergencyStop();
            // 状态重申：只有当前没在跑才起步，已经在跑就一个字节都不动。
            // ESP32 每 200ms 发一次，所以 ACT_START 丢了也能自动补上。
            else if (action == ACT_SYNC && !forwardSeq.running()) forwardSeq.start();
            break;
        case SUBSYS_TURN:
            // 转向是一次性 ~1 秒的动作，不做状态重申（重申会把它无限重启）。
            if (action == ACT_START)          turnSeq.start(true);
            else if (action == ACT_START_ALT) turnSeq.start(false);
            else if (action == ACT_BALANCE)   turnSeq.forceBalance();
            else if (action == ACT_SYNC)      { /* 忽略 */ }
            else                              turnSeq.cancel();
            break;
        case SUBSYS_BUOYANCY: {
            uint8_t dir = BUOYANCY_STOP;
            if (action == ACT_START)          dir = BUOYANCY_ASCEND;
            else if (action == ACT_START_ALT) dir = BUOYANCY_DESCEND;
            else if (action == ACT_BALANCE)   dir = BUOYANCY_BALANCE;
            buoySeq.setIntent(dir, arg, millis());
            break;
        }
        default:
            break;
    }
}

static void executeCommand(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2) {
    switch (cmd) {
        case CMD_MOTION:
            handleMotion(data0, data1, data2);
            break;

        case CMD_SET_PARAM: {
            const uint16_t value = static_cast<uint16_t>(data1) | (static_cast<uint16_t>(data2) << 8);
            const bool ok = g_params.set(data0, value);
            Serial.print(F("[PARAM] set "));
            Serial.print(data0);
            Serial.print(F(" = "));
            Serial.print(value);
            Serial.println(ok ? F(" OK") : F(" 拒绝(编号越界或不允许为0)"));
            if (ok) {
                sendStatusToESP32(STATUS_PARAM, data0, data1, data2);   // 回读确认
            }
            break;
        }

        case CMD_SAVE_PARAMS:
            g_params.save();
            Serial.println(F("[PARAM] 已保存到 EEPROM"));
            break;

        case CMD_GET_PARAMS:
            g_params.print();
            // 同时回传给 ESP32：每个参数一帧，供上位机/控制台显示。
            for (uint8_t i = 0; i < P_COUNT; i++) {
                const uint16_t val = g_params[i];
                sendStatusToESP32(STATUS_PARAM, i,
                                  static_cast<uint8_t>(val & 0xFF),
                                  static_cast<uint8_t>(val >> 8));
            }
            break;

        case CMD_EMERGENCY_STOP:
            forwardSeq.emergencyStop();
            turnSeq.cancel();
            buoySeq.stop();
            motion.emergencyStopAll();
            break;

        default:
            Serial.print(F("Unknown command: 0x"));
            Serial.println(cmd, HEX);
            break;
    }
}

static void processESP32Command() {
    ProtocolFrame frame;
    while (protocolReceiver.poll(UART_FROM_ESP32, frame)) {
        executeCommand(frame.cmd, frame.data0, frame.data1, frame.data2);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    UART_FROM_ESP32.begin(UART_BAUD_RATE);

    const bool loaded = g_params.load();

    Serial.println(F("\n============================================"));
    Serial.println(F("  Arduino Minima Actuator Executor"));
    Serial.println(F(MINIMA_FW_VERSION));
    Serial.println(loaded ? F("  参数: 已从 EEPROM 载入") : F("  参数: EEPROM 无有效数据，用默认值"));
    Serial.println(F("============================================"));
    g_params.print();

    motion.begin();
    Serial.println(F("Actuator executor ready.\n"));
}

void loop() {
    const uint32_t loopStartUs = micros();
    const unsigned long now = millis();

    processESP32Command();

    // 本地时序推进 → 合成掩码 → 写引脚。
    forwardSeq.update(now);
    turnSeq.update(now);
    buoySeq.update(now);
    motion.applyMask(forwardSeq.mask() | turnSeq.mask());
    motion.applyBuoyancy(buoySeq.valves(), buoySeq.pwm());

    motion.update();   // 软件 PWM 波形

    if (now - tStatus >= STATUS_INTERVAL) {
        tStatus = now;
        // data0 bit7=1 表示 data1/data2 携带当前实际执行掩码。
        const uint16_t mask = motion.getMask();
        sendStatusToESP32(STATUS_MOTION, getMotionStatus() | 0x80,
                          static_cast<uint8_t>(mask & 0xFF),
                          static_cast<uint8_t>((mask >> 8) & 0xFF));
        motion.printStatus();
    }

    if (now - tHeartbeat >= HEARTBEAT_INTERVAL) {
        tHeartbeat = now;
        const uint16_t bad = protocolReceiver.badFrames;
        sendStatusToESP32(STATUS_HEARTBEAT,
                          static_cast<uint8_t>(bad & 0xFF),
                          static_cast<uint8_t>((bad >> 8) & 0xFF),
                          static_cast<uint8_t>(gLoopMaxUs / 1000 > 255 ? 255 : gLoopMaxUs / 1000));
        gLoopMaxUs = 0;
    }

    const uint32_t loopUs = micros() - loopStartUs;
    if (loopUs > gLoopMaxUs) gLoopMaxUs = loopUs;
}
