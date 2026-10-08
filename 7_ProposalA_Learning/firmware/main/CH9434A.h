/**********************************************************************
 * CH9434A.h
 *
 * CH9434A SPI 转 4 路 UART 芯片驱动（原生 ESP-IDF spi_master 版）。
 * 保持与原 Arduino 驱动一致的寄存器定义和公开接口，SPI 时序用手动
 * 片选 + 逐字节 polling 传输复刻（含原驱动经验性的字节间微延时）。
 *********************************************************************/

#ifndef SQUID_CH9434A_H
#define SQUID_CH9434A_H

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"

// ── 寄存器地址 ──────────────────────────────────────────────────────
#define CH9434A_REG_RHR 0x00
#define CH9434A_REG_THR 0x00
#define CH9434A_REG_IER 0x01
#define CH9434A_REG_IIR 0x02
#define CH9434A_REG_FCR 0x02
#define CH9434A_REG_LCR 0x03
#define CH9434A_REG_MCR 0x04
#define CH9434A_REG_LSR 0x05
#define CH9434A_REG_MSR 0x06
#define CH9434A_REG_DLL 0x00
#define CH9434A_REG_DLM 0x01

// ── LCR 位 ──────────────────────────────────────────────────────────
#define CH9434A_LCR_DLAB   0x80
#define CH9434A_LCR_SBC    0x40
#define CH9434A_LCR_EPAR   0x10
#define CH9434A_LCR_PARITY 0x08
#define CH9434A_LCR_STOPB  0x04
#define CH9434A_LCR_8N1    0x03
#define CH9434A_LCR_7N1    0x02
#define CH9434A_LCR_6N1    0x01
#define CH9434A_LCR_5N1    0x00

// ── LSR 位 ──────────────────────────────────────────────────────────
#define CH9434A_LSR_TEMT 0x40
#define CH9434A_LSR_THRE 0x20
#define CH9434A_LSR_BI   0x10
#define CH9434A_LSR_FE   0x08
#define CH9434A_LSR_PE   0x04
#define CH9434A_LSR_OE   0x02
#define CH9434A_LSR_DR   0x01

// ── FCR 位 ──────────────────────────────────────────────────────────
#define CH9434A_FCR_ENABLE   0x01
#define CH9434A_FCR_RX_RESET 0x02
#define CH9434A_FCR_TX_RESET 0x04

// ── IER 位 ──────────────────────────────────────────────────────────
#define CH9434A_IER_RX  0x01
#define CH9434A_IER_TX  0x02
#define CH9434A_IER_LSR 0x04
#define CH9434A_IER_MSR 0x08

#define CH9434A_NUM_UARTS      4
#define CH9434A_UART_BASE_STEP 0x10

class CH9434A {
public:
    CH9434A(gpio_num_t csPin, gpio_num_t intPin = GPIO_NUM_NC);

    bool begin(uint32_t spiFreq = 10000000);
    bool config(uint8_t uartNum, uint32_t baudrate, uint8_t config = CH9434A_LCR_8N1);

    void   write(uint8_t uartNum, uint8_t data);
    size_t write(uint8_t uartNum, const uint8_t* buffer, size_t length);
    size_t print(uint8_t uartNum, const char* str);
    size_t println(uint8_t uartNum, const char* str);

    uint8_t read(uint8_t uartNum);
    size_t  read(uint8_t uartNum, uint8_t* buffer, size_t maxLength);
    int     available(uint8_t uartNum);
    void    flush(uint8_t uartNum);

    uint8_t getLineStatus(uint8_t uartNum);
    uint8_t getInterruptStatus(uint8_t uartNum);

    void enableInterrupt(uint8_t uartNum, uint8_t flags);
    void disableInterrupt(uint8_t uartNum);

    bool isTxEmpty(uint8_t uartNum);
    void waitTxComplete(uint8_t uartNum, uint32_t timeout = 100);

private:
    gpio_num_t       _csPin;
    gpio_num_t       _intPin;
    spi_device_handle_t _spi;
    bool             _spiReady;

    uint8_t spiTransfer(uint8_t out);
    void    writeReg(uint8_t uartNum, uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t uartNum, uint8_t reg);
    void    setBaudrate(uint8_t uartNum, uint32_t baudrate);
};

#endif  // SQUID_CH9434A_H
