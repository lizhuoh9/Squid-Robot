/**********************************************************************
 * CH9434A.cpp
 *
 * CH9434A 底层 SPI 读写（原生 spi_master）。
 * SPI2/FSPI 总线由本驱动独占（SD 卡用 SPI3）。片选用 GPIO 手动控制，
 * 以便在两个字节之间插入原驱动经验性的微延时。
 *********************************************************************/

#include "CH9434A.h"

#include <cstring>

#include "Board.h"
#include "Sys.h"

CH9434A::CH9434A(gpio_num_t csPin, gpio_num_t intPin)
    : _csPin(csPin), _intPin(intPin), _spi(nullptr), _spiReady(false) {}

bool CH9434A::begin(uint32_t spiFreq) {
    // 片选脚输出、默认拉高。
    gpio_config_t csCfg = {};
    csCfg.pin_bit_mask = 1ULL << _csPin;
    csCfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&csCfg);
    gpio_set_level(_csPin, 1);

    // 可选中断脚：上拉输入。
    if (_intPin != GPIO_NUM_NC) {
        gpio_config_t intCfg = {};
        intCfg.pin_bit_mask = 1ULL << _intPin;
        intCfg.mode = GPIO_MODE_INPUT;
        intCfg.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&intCfg);
    }

    // 初始化 SPI2 总线（仅 CH9434A 使用）。
    if (!_spiReady) {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = SPI_MOSI;
        buscfg.miso_io_num = SPI_MISO;
        buscfg.sclk_io_num = SPI_SCK;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 32;
        if (spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED) != ESP_OK) {
            return false;
        }

        spi_device_interface_config_t devcfg = {};
        devcfg.clock_speed_hz = static_cast<int>(spiFreq);
        devcfg.mode = 0;                 // SPI_MODE0
        devcfg.spics_io_num = -1;        // 片选手动控制
        devcfg.queue_size = 1;
        devcfg.flags = 0;
        if (spi_bus_add_device(SPI2_HOST, &devcfg, &_spi) != ESP_OK) {
            return false;
        }
        _spiReady = true;
    }

    // 上电后先等待芯片稳定。
    delay(100);

    // 逐路初始化 4 个 UART 通道。
    for (uint8_t i = 0; i < CH9434A_NUM_UARTS; i++) {
        writeReg(i, CH9434A_REG_IER, 0x00);
        writeReg(i, CH9434A_REG_FCR,
                 CH9434A_FCR_ENABLE | CH9434A_FCR_RX_RESET | CH9434A_FCR_TX_RESET);
        writeReg(i, CH9434A_REG_LCR, CH9434A_LCR_8N1);
    }

    // 读 UART0 的 LSR 做基础连通性检查。
    const uint8_t lsr = readReg(0, CH9434A_REG_LSR);
    return (lsr != 0xFF && lsr != 0x00 && lsr != 0x10);
}

bool CH9434A::config(uint8_t uartNum, uint32_t baudrate, uint8_t config) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return false;
    }
    setBaudrate(uartNum, baudrate);
    writeReg(uartNum, CH9434A_REG_LCR, config);
    writeReg(uartNum, CH9434A_REG_FCR,
             CH9434A_FCR_ENABLE | CH9434A_FCR_RX_RESET | CH9434A_FCR_TX_RESET);
    return true;
}

void CH9434A::write(uint8_t uartNum, uint8_t data) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return;
    }
    // 正常情况下 THRE 立刻就绪；SPI 或通道异常时这里是主循环的阻塞点，
    // 所以超时压到 20ms（三路超声波轮询最坏 60ms，而不是 300ms）。
    const uint32_t timeout = millis() + 20;
    while (!(readReg(uartNum, CH9434A_REG_LSR) & CH9434A_LSR_THRE)) {
        if (millis() > timeout) {
            return;
        }
        yieldCpu();
    }
    writeReg(uartNum, CH9434A_REG_THR, data);
}

size_t CH9434A::write(uint8_t uartNum, const uint8_t* buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        write(uartNum, buffer[i]);
    }
    return length;
}

size_t CH9434A::print(uint8_t uartNum, const char* str) {
    return write(uartNum, reinterpret_cast<const uint8_t*>(str), strlen(str));
}

size_t CH9434A::println(uint8_t uartNum, const char* str) {
    const size_t n = print(uartNum, str);
    write(uartNum, '\r');
    write(uartNum, '\n');
    return n + 2;
}

uint8_t CH9434A::read(uint8_t uartNum) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return 0;
    }
    if (!(readReg(uartNum, CH9434A_REG_LSR) & CH9434A_LSR_DR)) {
        return 0;
    }
    return readReg(uartNum, CH9434A_REG_RHR);
}

size_t CH9434A::read(uint8_t uartNum, uint8_t* buffer, size_t maxLength) {
    size_t count = 0;
    while (count < maxLength) {
        if (!(readReg(uartNum, CH9434A_REG_LSR) & CH9434A_LSR_DR)) {
            break;
        }
        buffer[count++] = readReg(uartNum, CH9434A_REG_RHR);
    }
    return count;
}

int CH9434A::available(uint8_t uartNum) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return 0;
    }
    return (readReg(uartNum, CH9434A_REG_LSR) & CH9434A_LSR_DR) ? 1 : 0;
}

void CH9434A::flush(uint8_t uartNum) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return;
    }
    writeReg(uartNum, CH9434A_REG_FCR, CH9434A_FCR_ENABLE | CH9434A_FCR_RX_RESET);
}

uint8_t CH9434A::getLineStatus(uint8_t uartNum) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return 0;
    }
    return readReg(uartNum, CH9434A_REG_LSR);
}

uint8_t CH9434A::getInterruptStatus(uint8_t uartNum) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return 0;
    }
    return readReg(uartNum, CH9434A_REG_IIR);
}

void CH9434A::enableInterrupt(uint8_t uartNum, uint8_t flags) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return;
    }
    writeReg(uartNum, CH9434A_REG_IER, flags);
}

void CH9434A::disableInterrupt(uint8_t uartNum) {
    if (uartNum >= CH9434A_NUM_UARTS) {
        return;
    }
    writeReg(uartNum, CH9434A_REG_IER, 0x00);
}

bool CH9434A::isTxEmpty(uint8_t uartNum) {
    return (getLineStatus(uartNum) & CH9434A_LSR_TEMT) != 0;
}

void CH9434A::waitTxComplete(uint8_t uartNum, uint32_t timeout) {
    const uint32_t start = millis();
    while (!isTxEmpty(uartNum)) {
        if (millis() - start > timeout) {
            break;
        }
        yieldCpu();
    }
}

// ── 底层 SPI ─────────────────────────────────────────────────────────
uint8_t CH9434A::spiTransfer(uint8_t out) {
    uint8_t in = 0;
    spi_transaction_t t = {};
    t.length = 8;                 // 8 bit
    t.tx_buffer = &out;
    t.rx_buffer = &in;
    spi_device_polling_transmit(_spi, &t);
    return in;
}

void CH9434A::writeReg(uint8_t uartNum, uint8_t reg, uint8_t value) {
    const uint8_t addr = reg + (CH9434A_UART_BASE_STEP * uartNum);

    gpio_set_level(_csPin, 0);
    spiTransfer(0x80 | addr);     // 写命令，最高位为 1
    delayMicroseconds(1);
    spiTransfer(value);
    delayMicroseconds(1);
    delayMicroseconds(1);
    delayMicroseconds(1);
    gpio_set_level(_csPin, 1);
}

uint8_t CH9434A::readReg(uint8_t uartNum, uint8_t reg) {
    const uint8_t addr = reg + (CH9434A_UART_BASE_STEP * uartNum);

    gpio_set_level(_csPin, 0);
    spiTransfer(addr);            // 读命令，最高位为 0
    delayMicroseconds(1);
    delayMicroseconds(1);
    delayMicroseconds(1);
    const uint8_t value = spiTransfer(0xFF);
    delayMicroseconds(1);
    gpio_set_level(_csPin, 1);
    return value;
}

void CH9434A::setBaudrate(uint8_t uartNum, uint32_t baudrate) {
    const uint32_t clockFreq = 32000000;
    const uint16_t divisor = clockFreq / (8 * baudrate);

    const uint8_t lcr = readReg(uartNum, CH9434A_REG_LCR);
    writeReg(uartNum, CH9434A_REG_LCR, lcr | CH9434A_LCR_DLAB);
    writeReg(uartNum, CH9434A_REG_DLL, divisor & 0xFF);
    writeReg(uartNum, CH9434A_REG_DLM, (divisor >> 8) & 0xFF);
    writeReg(uartNum, CH9434A_REG_LCR, lcr & ~CH9434A_LCR_DLAB);
}
