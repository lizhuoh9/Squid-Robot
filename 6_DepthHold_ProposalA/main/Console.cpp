/**********************************************************************
 * Console.cpp
 *********************************************************************/

#include "Console.h"

#include "driver/uart.h"
#include "driver/uart_vfs.h"

namespace {
constexpr uart_port_t CONSOLE_UART = UART_NUM_0;
constexpr int         RX_BUF_SIZE  = 512;
// 控制台发送缓冲：stdout 经驱动环形缓冲异步发出，打印不再逐字节忙等阻塞主循环
// （原来 tx=0 时每秒传感器面板 ~600B@115200 会把主循环卡住 ~50ms+）。
constexpr int         TX_BUF_SIZE  = 4096;
bool                  g_installed   = false;
}  // namespace

void consoleBegin() {
    if (g_installed) return;

    // UART0 在启动阶段已由 bootloader/console 配置好引脚和波特率，
    // 这里只装驱动用于中断式接收；TX 继续走默认控制台。
    const uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(CONSOLE_UART, &cfg);
    uart_driver_install(CONSOLE_UART, RX_BUF_SIZE, TX_BUF_SIZE, 0, nullptr, 0);
    // 让 stdout/printf/ESP_LOG 都走驱动缓冲（中断发送），而不是 ROM 轮询忙等。
    uart_vfs_dev_use_driver(CONSOLE_UART);
    g_installed = true;
}

bool consoleTxReady() { return g_installed; }

void consoleWrite(const uint8_t* data, size_t len) {
    if (!g_installed || !data || len == 0) return;
    // 每次写入在驱动环形缓冲里另占 ~8B 头部，留余量；装不下就丢，绝不阻塞主循环。
    size_t freeSz = 0;
    if (uart_get_tx_buffer_free_size(CONSOLE_UART, &freeSz) != ESP_OK || freeSz < len + 16) {
        return;
    }
    uart_write_bytes(CONSOLE_UART, reinterpret_cast<const char*>(data), len);
}

int consoleAvailable() {
    size_t n = 0;
    if (uart_get_buffered_data_len(CONSOLE_UART, &n) != ESP_OK) return 0;
    return static_cast<int>(n);
}

int consoleReadByte() {
    uint8_t b = 0;
    int n = uart_read_bytes(CONSOLE_UART, &b, 1, 0);
    return n == 1 ? static_cast<int>(b) : -1;
}
