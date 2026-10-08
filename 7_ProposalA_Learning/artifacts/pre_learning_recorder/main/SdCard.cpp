/**********************************************************************
 * SdCard.cpp
 *********************************************************************/

#include "SdCard.h"

#include <cstdio>
#include <cstring>

#include "Board.h"
#include "Output.h"
#include "Sys.h"
#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"
#include "sd_protocol_defs.h"
#include "sdmmc_cmd.h"

const char* const SD_MOUNT_POINT = "/sdcard";

namespace {
sdmmc_card_t* g_card = nullptr;
bool          g_mounted = false;
bool          g_busReady = false;

bool ensureBus() {
    if (g_busReady) return true;
    // SPI3/HSPI 总线（独立于 CH9434A 的 SPI2）。
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SD_SPI_MOSI;
    buscfg.miso_io_num = SD_SPI_MISO;
    buscfg.sclk_io_num = SD_SPI_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4092;
    const esp_err_t err = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;
    g_busReady = true;
    return true;
}

void setPullups(bool on) {
    for (gpio_num_t pin : {SD_SPI_MISO, SD_SPI_MOSI, SD_SPI_CS}) {
        gpio_set_pull_mode(pin, on ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
    }
}

// 一次挂载尝试。freqKhz = 初始化完成后的工作时钟（探测阶段固定 400kHz）。
esp_err_t mountOnce(bool pullups, uint32_t settleMs, int freqKhz) {
    if (!ensureBus()) return ESP_FAIL;
    setPullups(pullups);
    if (settleMs) delay(settleMs);

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    host.max_freq_khz = freqKhz;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_SPI_CS;
    slot.host_id = SPI3_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mountCfg = {};
    mountCfg.format_if_mount_failed = false;
    mountCfg.max_files = 6;
    mountCfg.allocation_unit_size = 16 * 1024;

    const esp_err_t err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mountCfg, &g_card);
    if (err == ESP_OK) g_mounted = true;
    return err;
}

void unmount() {
    if (!g_mounted) return;
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, g_card);
    g_card = nullptr;
    g_mounted = false;
}
}  // namespace

bool sdCardMount() {
    // This separate recorder project is deliberately SD-free.
    return false;
    if (g_mounted) return true;
    // 内部上拉（MISO/MOSI/CS，SD 规范要求这些线有上拉）+ 失败重试 3 次：
    // 冷启动时卡可能还没完成内部上电，第一次 ACMD41 会超时。
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (mountOnce(true, attempt == 0 ? 0 : 200, SDMMC_FREQ_DEFAULT) == ESP_OK) return true;
    }
    return false;
}

namespace {
// ── 原始 SPI 探测：绕过 IDF sdmmc 协议栈，手动收发 SD 命令并打印卡的原始应答 ──
spi_device_handle_t g_raw = nullptr;

void rawXfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    spi_transaction_t t = {};
    t.length = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(g_raw, &t);
}

// 发一条 6 字节命令，随后读 respLen 字节（含等待 R1 的 NCR 间隔），打印十六进制。
void rawCmd(const char* name, uint8_t cmd, uint32_t arg, uint8_t crc, size_t readLen, bool keepCs = false) {
    uint8_t tx[24];
    uint8_t rx[24] = {};
    size_t n = 0;
    tx[n++] = 0xFF;
    tx[n++] = static_cast<uint8_t>(0x40 | cmd);
    tx[n++] = static_cast<uint8_t>(arg >> 24);
    tx[n++] = static_cast<uint8_t>(arg >> 16);
    tx[n++] = static_cast<uint8_t>(arg >> 8);
    tx[n++] = static_cast<uint8_t>(arg);
    tx[n++] = crc;
    const size_t head = n;
    for (size_t i = 0; i < readLen && n < sizeof(tx); ++i) tx[n++] = 0xFF;
    gpio_set_level(SD_SPI_CS, 0);
    rawXfer(tx, rx, n);
    if (!keepCs) {
        gpio_set_level(SD_SPI_CS, 1);
        const uint8_t ff = 0xFF;
        uint8_t dummy;
        rawXfer(&ff, &dummy, 1);
    }
    char line[128];
    int p = snprintf(line, sizeof(line), "[SD原始] %-7s ->", name);
    for (size_t i = head; i < n && p < static_cast<int>(sizeof(line)) - 4; ++i) {
        p += snprintf(line + p, sizeof(line) - p, " %02X", rx[i]);
    }
    g_usb->println(line);
}

void rawProbe() {
    if (!ensureBus()) {
        g_usb->println("[SD原始] SPI3 总线初始化失败");
        return;
    }
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = 400 * 1000;
    dev.mode = 0;
    dev.spics_io_num = -1;   // 手动控制 CS
    dev.queue_size = 1;
    if (spi_bus_add_device(SPI3_HOST, &dev, &g_raw) != ESP_OK) {
        g_usb->println("[SD原始] 添加 SPI 设备失败（总线被占用？）");
        return;
    }
    gpio_config_t cs = {};
    cs.pin_bit_mask = 1ULL << SD_SPI_CS;
    cs.mode = GPIO_MODE_OUTPUT;
    gpio_config(&cs);
    gpio_set_level(SD_SPI_CS, 1);
    setPullups(true);

    // ≥74 个时钟，CS 高
    uint8_t ff[10], dummy[10];
    memset(ff, 0xFF, sizeof(ff));
    rawXfer(ff, dummy, sizeof(ff));

    g_usb->println("[SD原始] 期望: CMD0=01  CMD8=01 00 00 01 AA  CMD58=01/00+OCR  CMD55=01  ACMD41=01...最终00");
    rawCmd("CMD0", 0, 0, 0x95, 8);
    rawCmd("CMD8", 8, 0x1AA, 0x87, 10);
    rawCmd("CMD58", 58, 0, 0xFD, 10);
    for (int i = 0; i < 6; ++i) {
        rawCmd("CMD55", 55, 0, 0x65, 8);
        rawCmd("ACMD41", 41, 0x40000000, 0x77, 8);
        delay(50);
    }
    rawCmd("CMD58", 58, 0, 0xFD, 10);

    spi_bus_remove_device(g_raw);
    g_raw = nullptr;
}
}  // namespace

bool sdCardDiagnose() {
    // Do not probe SPI/card hardware from the legacy console command.
    return false;
    unmount();
    rawProbe();
    struct Combo { bool pullups; uint32_t settleMs; int freqKhz; const char* note; };
    static const Combo combos[] = {
        {false,   0, SDMMC_FREQ_DEFAULT, "原配置(无上拉,20MHz)"},
        {true,    0, SDMMC_FREQ_DEFAULT, "内部上拉"},
        {true,  500, SDMMC_FREQ_DEFAULT, "上拉+等500ms"},
        {true,  500, 4000,               "上拉+等500ms+4MHz"},
        {true, 1000, SDMMC_FREQ_PROBING, "上拉+等1s+400kHz"},
        {false, 1000, 4000,              "无上拉+等1s+4MHz"},
    };
    unmount();
    g_usb->println("[SD诊断] 开始：每组参数尝试挂载一次（探测阶段固定 400kHz）");
    int firstOk = -1;
    for (size_t i = 0; i < sizeof(combos) / sizeof(combos[0]); ++i) {
        const Combo& c = combos[i];
        const uint32_t t0 = millis();
        const esp_err_t err = mountOnce(c.pullups, c.settleMs, c.freqKhz);
        const uint32_t dt = millis() - t0;
        if (err == ESP_OK) {
            g_usb->printf("[SD诊断] #%u %-22s OK  容量=%u MB  类型=%s  (%u ms)\r\n",
                          static_cast<unsigned>(i + 1), c.note,
                          static_cast<unsigned>(sdCardSizeBytes() / (1024ULL * 1024ULL)),
                          g_card && (g_card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC",
                          static_cast<unsigned>(dt));
            if (firstOk < 0) {
                firstOk = static_cast<int>(i);
            }
            unmount();
        } else {
            g_usb->printf("[SD诊断] #%u %-22s 失败 0x%x %s  (%u ms)\r\n",
                          static_cast<unsigned>(i + 1), c.note, static_cast<unsigned>(err),
                          esp_err_to_name(err), static_cast<unsigned>(dt));
        }
    }
    if (firstOk >= 0) {
        const Combo& c = combos[firstOk];
        mountOnce(c.pullups, c.settleMs, c.freqKhz);
        g_usb->printf("[SD诊断] 结论：有组合可用，已按 #%d 重新挂载。\r\n", firstOk + 1);
        return g_mounted;
    }
    g_usb->println("[SD诊断] 结论：全部失败——卡在 ACMD41 初始化阶段不就绪，软件参数无效，指向卡/供电/焊接。");
    return false;
}

bool sdCardMounted() { return g_mounted; }

uint64_t sdCardSizeBytes() {
    if (!g_card) return 0;
    return static_cast<uint64_t>(g_card->csd.capacity) * g_card->csd.sector_size;
}

char* sdFsPath(char* dst, size_t dstLen, const char* logicalPath) {
    if (!logicalPath || logicalPath[0] == '\0' ||
        (logicalPath[0] == '/' && logicalPath[1] == '\0')) {
        snprintf(dst, dstLen, "%s", SD_MOUNT_POINT);
        return dst;
    }
    if (logicalPath[0] == '/') {
        snprintf(dst, dstLen, "%s%s", SD_MOUNT_POINT, logicalPath);
    } else {
        snprintf(dst, dstLen, "%s/%s", SD_MOUNT_POINT, logicalPath);
    }
    return dst;
}
