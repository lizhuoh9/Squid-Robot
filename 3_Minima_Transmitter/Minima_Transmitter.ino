/**********************************************************************
 * Arduino HC-12 透明串口桥  (Arduino UNO R4 Minima)
 *
 * 架构：
 *   PC  ──USB Serial──►  Minima  ──HC-12 无线──►  ESP32 机器人
 *   PC  ◄─USB Serial──   Minima  ◄─HC-12 无线──   ESP32 机器人
 *
 * 正常使用：
 *   完全透明转发。PC 发送的所有内容原样经 HC-12 转发给机器人，
 *   机器人的所有回传也原样转发回 PC。
 *
 * ACK / 重发 / 去重（v2.0）：
 *   每条命令带一个序号发出："<命令>#<序号>"，序号 0~99 循环。
 *   按行扫描回传流，整行等于 "[OK]<命令>#<序号>" 才算送达（400ms 超时，最多 5 次）。
 *
 *   ⚠ 三件事必须同时成立，缺一条就会出问题：
 *     1) 整行比对。只匹配 "[OK]" 四个字节的话，一条丢在空中的 w 会被随后的
 *        "[OK]hb" 顶掉 —— 桥以为送达、不再重发，命令就此消失。
 *     2) 带序号。w/j/k 都是切换式命令：命令到了、只是回执丢了的话，重发会把
 *        刚起来的动作又关掉。机器人靠序号认出重发，只补回执、不重复执行。
 *     3) 有了 1)+2)，超时才敢从 1500ms 压到 400ms。实测命令往返 50~150ms，
 *        遥测压缩后链路 98% 时间是空的，400ms 足够且丢一条能很快补回来。
 *
 * 本端信道配置命令（PC 发 → Minima 拦截处理，不转发）：
 *   HC001 ~ HC127   将本端 HC-12 切换到指定信道
 *   ?HC             进 AT 模式读回本端 HC-12 参数（AT / AT+V / AT+RX），
 *                   用于验证 HC-12 接线与当前信道/波特率，不经无线转发。
 *   ?ID             回一行识别帧（仅本地 USB 串口，不经 HC-12）：
 *                   [BRIDGE]SQUID-HC12-BRIDGE v2.0
 *                   供 PC 控制台自动发现串口用，不占无线带宽。
 *
 * 机器人端信道配置命令（PC 发 → 透传给机器人）：
 *   ESP001 ~ ESP127  机器人收到后自行配置其 HC-12 信道
 *
 * ⚠ 双端同步信道的正确顺序：
 *   1. 先发 ESP025  → 机器人切换到新信道
 *   2. 再发 HC025   → 本端切换到新信道 → 链路恢复
 *
 * HC-12 接线（Arduino UNO R4 Minima）：
 *   HC12_USE_HW_SERIAL1 = 0（默认，现有接线）：
 *     HC-12 TX  → D2  (SoftwareSerial RX)
 *     HC-12 RX  → D3  (SoftwareSerial TX)
 *   HC12_USE_HW_SERIAL1 = 1（硬件串口 Serial1 = SCI2，需改线）：
 *     HC-12 TX  → D0
 *     HC-12 RX  → D1
 *   RA4M1 没有引脚交叉矩阵，D2 根本不具备串口功能，所以升到 38400 以上
 *   必须改到 D0/D1。软件串口收发要关中断、靠死等时序，9600 以上易丢字节。
 *   HC-12 SET → D4
 *   HC-12 VCC → 5V
 *   HC-12 GND → GND
 *********************************************************************/

// 1 = HC-12 接硬件串口 Serial1(D0/D1)；0 = 维持软件串口(D2/D3)。改线后再置 1。
#define HC12_USE_HW_SERIAL1 1

#if !HC12_USE_HW_SERIAL1
#include <SoftwareSerial.h>
#endif

#define HC12_RX_PIN  2
#define HC12_TX_PIN  3
#define HC12_SET_PIN 4
#define PC_BAUD      115200
#define HC12_BAUD    9600
#define PC_BUF_MAX   64

// 命令往返实测 50~150ms（PC->桥 115200，空口 9600 + HC-12 自身延迟，机器人 <5ms）。
// 遥测压到 40 字节/2s 之后链路几乎全空，400ms 超时既不会误判也能快速补发。
constexpr uint32_t ACK_TIMEOUT_MS = 400;
constexpr uint8_t  MAX_RETRIES    = 5;

// 识别帧：PC 控制台发 "?ID" 即回这一行，用来在多个串口里认出本机。
static const char BRIDGE_ID[] = "[BRIDGE]SQUID-HC12-BRIDGE v2.0";

// ACK 比对：机器人执行完一条 HC-12 命令后回 "[OK]<命令回显>"（小写）。
// 必须整行对上才算这条命令的 ACK —— 只认 "[OK]" 四个字节是不行的，理由见 loop()。
static const char ACK_PREFIX[]   = "[OK]";
static const uint8_t RX_LINE_MAX = 72;   // 回执行最长这么多字节，超了肯定不是回执

#if HC12_USE_HW_SERIAL1
#define hc12 Serial1
#else
SoftwareSerial hc12(HC12_RX_PIN, HC12_TX_PIN);
#endif

String   pcBuffer   = "";
String   pendingCmd = "";
uint32_t pendingMs  = 0;
uint8_t  retryCount = 0;
bool     waitingAck = false;
String   rxLine     = "";      // 正在接收的回传行（只用于 ACK 比对，转发不受影响）
bool     rxLineBad  = false;   // 本行超长/被截断，不参与 ACK 比对
uint8_t  cmdSeq     = 0;       // 命令序号 0~99，每条新命令 +1；重发沿用同一个号

// ────────────────────────────────────────
// 发送命令并启动 ACK 等待
// ────────────────────────────────────────
void sendCmd(const String& cmd) {
    // 先 trim 再发：机器人收到后也会 trim + 转小写才回显，这边不 trim 的话
    // 一个多打的空格就会让整行 ACK 比对对不上，白白触发重发。
    String c = cmd;
    c.trim();
    if (c.length() == 0) return;

    cmdSeq = (cmdSeq + 1) % 100;
    pendingCmd = c + "#" + String(cmdSeq);   // 重发时原样再发这一整串

    hc12.println(pendingCmd);
    pendingMs  = millis();
    retryCount = 0;
    waitingAck = true;
}

// 只转发、不等 ACK（心跳、信道切换这类拿不到回执的命令）。
void sendNoAck(const String& cmd) {
    hc12.println(cmd);
}

// ────────────────────────────────────────
// 将本端 HC-12 切换到指定信道（AT 模式）
// ────────────────────────────────────────
void configHC12Channel(const String& channel) {
    waitingAck = false;
    rxLine     = "";
    rxLineBad  = false;

    Serial.print(F("[HC-12] Setting channel to "));
    Serial.print(channel);
    Serial.print(F(" ..."));

    digitalWrite(HC12_SET_PIN, LOW);
    delay(200);

    hc12.print(F("AT+C"));
    hc12.println(channel);

    String response = "";
    const uint32_t start = millis();
    while (millis() - start < 1000) {
        if (hc12.available()) {
            response += (char)hc12.read();
        }
    }

    digitalWrite(HC12_SET_PIN, HIGH);
    delay(200);

    response.trim();
    if (response.indexOf("OK") >= 0) {
        Serial.print(F(" OK (ch."));
        Serial.print(channel);
        Serial.println(F(")"));
    } else {
        Serial.print(F(" FAILED"));
        if (response.length() > 0) {
            Serial.print(F(": "));
            Serial.print(response);
        } else {
            Serial.print(F(": no response — check 5V power and SET wiring"));
        }
        Serial.println();
    }
}

// ────────────────────────────────────────
// 读回本端 HC-12 参数（AT 模式）：验证接线 + 查看信道/波特率/功率/模式
// ────────────────────────────────────────
void queryHC12() {
    waitingAck = false;
    rxLine     = "";
    rxLineBad  = false;

    Serial.println(F("[HC-12] 进入 AT 模式查询参数..."));
    digitalWrite(HC12_SET_PIN, LOW);
    delay(200);

    const char* cmds[] = {"AT", "AT+V", "AT+RX"};
    for (uint8_t i = 0; i < 3; i++) {
        while (hc12.available()) hc12.read();   // 清掉上一条的残留
        hc12.println(cmds[i]);
        Serial.print(F("  "));
        Serial.print(cmds[i]);
        Serial.print(F(" -> "));
        String resp = "";
        const uint32_t start = millis();
        while (millis() - start < 600) {
            if (hc12.available()) resp += (char)hc12.read();
        }
        resp.trim();
        resp.replace("\r\n", " | ");
        resp.replace("\n", " | ");
        Serial.println(resp.length() ? resp : String(F("(无响应 — 检查 HC-12 接线/供电/SET 脚)")));
    }

    digitalWrite(HC12_SET_PIN, HIGH);
    delay(200);
    Serial.println(F("[HC-12] 已退回透传模式"));
}

// ────────────────────────────────────────
// 初始化
// ────────────────────────────────────────
void setup() {
    pinMode(HC12_SET_PIN, OUTPUT);
    digitalWrite(HC12_SET_PIN, HIGH);

    Serial.begin(PC_BAUD);
    while (!Serial);
    hc12.begin(HC12_BAUD);

    Serial.println(F("============================================"));
    Serial.println(F("   Squid Robot  —  Minima HC-12 Bridge"));
    Serial.println(F("--------------------------------------------"));
    Serial.println(F(" Channel pairing"));
    Serial.println(F("   HC025   set THIS  end to ch.025"));
    Serial.println(F("   ESP025  set ROBOT end to ch.025"));
    Serial.println(F("   Order : ESP025 first, HC025 second"));
    Serial.println(F("--------------------------------------------"));
    Serial.println(F(" ACK: whole-line match of \"[OK]<cmd>#<seq>\""));
    Serial.println(F(" Retry: 400ms timeout, max 5 retries (robot dedupes by seq)"));
    Serial.println(F("============================================"));
    Serial.println(BRIDGE_ID);   // 开机也报一次，控制台可直接认出
}

// ────────────────────────────────────────
// 主循环
// ────────────────────────────────────────
void loop() {
    // ── PC → HC-12 ──────────────────────────────────────────────────
    while (Serial.available()) {
        const char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (pcBuffer.length() > 0) {
                // 识别帧查询：本地回一行，不经 HC-12，不影响 ACK 状态机。
                if (pcBuffer == "?HC" || pcBuffer == "?hc") {
                    queryHC12();
                    pcBuffer = "";
                    continue;
                }
                if (pcBuffer == "?ID" || pcBuffer == "?id") {
                    Serial.println(BRIDGE_ID);
                    pcBuffer = "";
                    continue;
                }
                // 心跳 hb：发了就算，不等 ACK、不重发。机器人不在线时可避免
                // 每 5s 三次重发白占半双工信道；丢一两个心跳无所谓。
                if (pcBuffer == "hb" || pcBuffer == "HB") {
                    sendNoAck(pcBuffer);
                    pcBuffer = "";
                    continue;
                }
                // ESP### 切信道：机器人收到后不回 [OK]，而且立刻换到新信道，
                // 老信道上再重发 3 次是纯浪费（还会被误判成 [NoACK]）。只发一次。
                if (pcBuffer.length() == 6 &&
                    (pcBuffer[0] == 'E' || pcBuffer[0] == 'e') &&
                    (pcBuffer[1] == 'S' || pcBuffer[1] == 's') &&
                    (pcBuffer[2] == 'P' || pcBuffer[2] == 'p') &&
                    isDigit(pcBuffer[3]) && isDigit(pcBuffer[4]) && isDigit(pcBuffer[5])) {
                    sendNoAck(pcBuffer);
                    Serial.print(F("[Bridge] "));
                    Serial.print(pcBuffer);
                    Serial.println(F(" 已发出（不等 ACK）；机器人换信道后请发 HC### 跟上。"));
                    pcBuffer = "";
                    continue;
                }
                if (pcBuffer.length() == 5  &&
                    pcBuffer[0] == 'H'      &&
                    pcBuffer[1] == 'C'      &&
                    isDigit(pcBuffer[2])    &&
                    isDigit(pcBuffer[3])    &&
                    isDigit(pcBuffer[4])) {
                    configHC12Channel(pcBuffer.substring(2));
                } else {
                    sendCmd(pcBuffer);
                }
                pcBuffer = "";
            }
        } else {
            pcBuffer += c;
            if (pcBuffer.length() > PC_BUF_MAX) {
                Serial.println(F("[Bridge] Input overflow, buffer cleared."));
                pcBuffer = "";
            }
        }
    }

    // ── HC-12 → PC（字节级转发 + 整行 ACK 比对）──────────────────
    // 所有字节原样转发给 PC；同时把回传攒成一行，收到行尾再整行比对。
    //
    // 旧版本是滚动扫描 "[OK]" 这四个字节，不看后面跟的是哪条命令。可机器人对
    // **每一条** HC-12 命令都回 "[OK]<命令>"，心跳 hb 也回。于是：你按的 w 丢在
    // 空中 → 200ms 后机器人回了句 "[OK]hb" → 桥当成 w 的回执 → 打印 [ACK] w、
    // 清掉重发 → 这条命令永远送不到，控制台还显示"已确认"。必须整行对上。
    while (hc12.available()) {
        const char c = hc12.read();
        Serial.write(c);  // 原样转发

        if (c == '\n' || c == '\r') {
            if (waitingAck && !rxLineBad && rxLine.length() > 0) {
                String expect = String(ACK_PREFIX) + pendingCmd;
                expect.toLowerCase();       // 机器人回显的是转小写后的命令
                String got = rxLine;
                got.trim();
                got.toLowerCase();
                if (got == expect) {
                    waitingAck = false;
                    Serial.print(F("[ACK] "));
                    Serial.println(pendingCmd);
                }
            }
            rxLine    = "";
            rxLineBad = false;
            continue;
        }

        if (rxLine.length() < RX_LINE_MAX) {
            rxLine += c;
        } else {
            rxLineBad = true;   // 超长（传感器面板行等），本行不可能是回执
        }
    }

    // ── ACK 超时重发 ─────────────────────────────────────────────────
    if (waitingAck && (millis() - pendingMs) > ACK_TIMEOUT_MS) {
        if (retryCount < MAX_RETRIES) {
            retryCount++;
            hc12.println(pendingCmd);
            pendingMs = millis();
            Serial.print(F("[Retry "));
            Serial.print(retryCount);
            Serial.print('/');
            Serial.print(MAX_RETRIES);
            Serial.print(F("] "));
            Serial.println(pendingCmd);
        } else {
            Serial.print(F("[NoACK] "));
            Serial.println(pendingCmd);
            waitingAck = false;
        }
    }
}
