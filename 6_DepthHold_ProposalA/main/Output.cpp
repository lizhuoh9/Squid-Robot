/**********************************************************************
 * Output.cpp
 *
 * Output 基类便捷方法 + 三种输出目标（USB / HC-12 / 两路分发）实现。
 *********************************************************************/

#include "Output.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "Board.h"
#include "Console.h"
#include "driver/uart.h"

// ── Output 便捷方法 ───────────────────────────────────────────────
void Output::print(const char* s) {
    if (s) writeBytes(reinterpret_cast<const uint8_t*>(s), strlen(s));
}

void Output::print(int v) {
    char buf[16];
    int n = snprintf(buf, sizeof(buf), "%d", v);
    if (n > 0) writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

void Output::print(unsigned int v) {
    char buf[16];
    int n = snprintf(buf, sizeof(buf), "%u", v);
    if (n > 0) writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

void Output::print(long v) {
    char buf[24];
    int n = snprintf(buf, sizeof(buf), "%ld", v);
    if (n > 0) writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

void Output::print(unsigned long v) {
    char buf[24];
    int n = snprintf(buf, sizeof(buf), "%lu", v);
    if (n > 0) writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

void Output::print(float v, int digits) {
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "%.*f", digits, static_cast<double>(v));
    if (n > 0) writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

void Output::print(double v, int digits) {
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "%.*f", digits, v);
    if (n > 0) writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

// println：与 Arduino 一致，行尾输出 "\r\n"（HC-12 桥端按行解析依赖 CRLF）。
void Output::println() {
    writeBytes(reinterpret_cast<const uint8_t*>("\r\n"), 2);
}
void Output::println(const char* s) { print(s); println(); }
void Output::println(int v)           { print(v); println(); }
void Output::println(unsigned int v)  { print(v); println(); }
void Output::println(long v)          { print(v); println(); }
void Output::println(unsigned long v) { print(v); println(); }
void Output::println(float v, int digits) { print(v, digits); println(); }

void Output::printf(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > static_cast<int>(sizeof(buf) - 1)) n = sizeof(buf) - 1;
    writeBytes(reinterpret_cast<const uint8_t*>(buf), n);
}

// ── StdoutOutput（USB/UART0）──────────────────────────────────────
size_t StdoutOutput::writeBytes(const uint8_t* data, size_t len) {
    if (!data || len == 0) return 0;
    if (!consoleTxReady()) {             // 早期启动：驱动未装，走默认控制台
        size_t w = fwrite(data, 1, len, stdout);
        fflush(stdout);
        return w;
    }
    // 驱动就绪：整段写入 UART0 发送缓冲。IDF 的 stdout VFS 会逐字符调用
    // uart_write_bytes（每字符一次加锁+环形缓冲头部），实测一屏面板仍卡 ~40ms，
    // 所以这里绕过 VFS。单独的 '\n' 补成 "\r\n"，与原 VFS 换行行为一致。
    size_t start = 0;
    for (size_t i = 0; i < len; ++i) {
        if (data[i] == '\n' && (i == 0 || data[i - 1] != '\r')) {
            consoleWrite(data + start, i - start);
            consoleWrite(reinterpret_cast<const uint8_t*>("\r\n"), 2);
            start = i + 1;
        }
    }
    consoleWrite(data + start, len - start);
    return len;
}

// ── Hc12Output（UART2）────────────────────────────────────────────
size_t Hc12Output::writeBytes(const uint8_t* data, size_t len) {
    if (!data || len == 0) return 0;
    // 绝不阻塞主循环：发送缓冲装不下就丢弃这段调试文字（HC-12 只 9600 波特）。
    // 每次写入在驱动环形缓冲里另占 ~8B 头部，留余量。
    size_t freeSz = 0;
    if (uart_get_tx_buffer_free_size(HC12_UART_PORT, &freeSz) != ESP_OK || freeSz < len + 16) {
        return 0;
    }
    int w = uart_write_bytes(HC12_UART_PORT, reinterpret_cast<const char*>(data), len);
    return w < 0 ? 0 : static_cast<size_t>(w);
}

// ── MultiOutput（TeeStream）───────────────────────────────────────
size_t MultiOutput::writeBytes(const uint8_t* data, size_t len) {
    if (_a) _a->writeBytes(data, len);
    if (_b) _b->writeBytes(data, len);
    return len;
}

// ── 全局实例 ──────────────────────────────────────────────────────
StdoutOutput g_usbOut;
Hc12Output   g_hc12Out;
MultiOutput  g_tee(&g_usbOut, &g_hc12Out);
Output*      g_usb = &g_usbOut;
Output*      g_dbg = &g_usbOut;   // HC-12 初始化前指向 USB，之后切到 g_tee
