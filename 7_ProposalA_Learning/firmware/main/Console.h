/**********************************************************************
 * Console.h
 *
 * USB/UART0 控制台输入。原固件用 Arduino Serial.available()/read()
 * 逐字节读命令，这里用 UART0 驱动做非阻塞读取。
 * 输出：驱动装好后 StdoutOutput 整段写入 UART0 驱动发送缓冲（非阻塞）；
 * 其余 printf/ESP_LOG 也经驱动缓冲发送（uart_vfs_dev_use_driver）。
 *********************************************************************/

#ifndef SQUID_CONSOLE_H
#define SQUID_CONSOLE_H

#include <cstddef>
#include <cstdint>

// 安装 UART0 驱动：非阻塞接收 + 4KB 发送缓冲。
void consoleBegin();

// UART0 驱动是否已安装（之前的早期打印仍走默认控制台）。
bool consoleTxReady();

// 把一段文字整块写入 UART0 发送缓冲（不逐字节忙等）；缓冲满则丢弃，永不阻塞。
void consoleWrite(const uint8_t* data, size_t len);

// 是否有可读字节（对应 Serial.available() > 0）。
int consoleAvailable();

// 读取一个字节；无数据返回 -1（对应 Serial.read()）。
int consoleReadByte();

#endif  // SQUID_CONSOLE_H
