/**********************************************************************
 * Output.h
 *
 * 原固件用 Arduino `Print` 抽象（Serial / HC12_SERIAL / TeeStream）
 * 把调试文本同时送往 USB 串口和 HC-12 无线。
 * 这里用一个轻量 Output 基类原生复刻，提供 Arduino 风格的
 * print / println / printf 便捷方法，让各模块逻辑几乎逐行照搬。
 *
 * 全局：
 *   g_usb  —— USB/UART0 控制台（对应 Arduino 的 Serial）
 *   g_dbg  —— 调试输出总线（USB + HC-12，对应 TeeStream）
 *********************************************************************/

#ifndef SQUID_OUTPUT_H
#define SQUID_OUTPUT_H

#include <cstddef>
#include <cstdint>
#include <string>

// ── 输出目标基类 ──────────────────────────────────────────────────
class Output {
public:
    virtual ~Output() = default;

    // 唯一需要子类实现的原语。
    virtual size_t writeBytes(const uint8_t* data, size_t len) = 0;

    size_t write(uint8_t c) { return writeBytes(&c, 1); }
    size_t write(const uint8_t* data, size_t len) { return writeBytes(data, len); }

    // ── Arduino 风格便捷方法 ──────────────────────────────────────
    void print(const char* s);
    void print(const std::string& s) { print(s.c_str()); }
    void print(char c) { write(static_cast<uint8_t>(c)); }
    void print(int v);
    void print(unsigned int v);
    void print(long v);
    void print(unsigned long v);
    void print(float v, int digits = 2);
    void print(double v, int digits = 2);

    void println();
    void println(const char* s);
    void println(const std::string& s) { println(s.c_str()); }
    void println(int v);
    void println(unsigned int v);
    void println(long v);
    void println(unsigned long v);
    void println(float v, int digits = 2);

    // 与 Arduino Print::printf 语义一致（内部 vsnprintf 到栈缓冲）。
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};

// ── USB/UART0 控制台（stdout）─────────────────────────────────────
class StdoutOutput : public Output {
public:
    size_t writeBytes(const uint8_t* data, size_t len) override;
};

// ── HC-12 无线串口（UART2）────────────────────────────────────────
class Hc12Output : public Output {
public:
    size_t writeBytes(const uint8_t* data, size_t len) override;
};

// ── 两路分发（对应 TeeStream）：USB + HC-12 ──────────────────────
class MultiOutput : public Output {
public:
    MultiOutput(Output* a, Output* b) : _a(a), _b(b) {}
    size_t writeBytes(const uint8_t* data, size_t len) override;

private:
    Output* _a;
    Output* _b;
};

// 全局输出实例（定义在 Output.cpp）。
extern StdoutOutput g_usbOut;      // USB/UART0
extern Hc12Output   g_hc12Out;     // HC-12
extern MultiOutput  g_tee;         // USB + HC-12 [+ web]
extern Output*      g_usb;         // 指向 g_usbOut（Arduino Serial 的替身）
extern Output*      g_dbg;         // 默认指向 g_usbOut，HC-12 就绪后指向 g_tee

#endif  // SQUID_OUTPUT_H
