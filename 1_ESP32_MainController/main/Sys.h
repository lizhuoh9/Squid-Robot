/**********************************************************************
 * Sys.h
 *
 * Arduino 时间/延时 API 的原生 ESP-IDF 等价实现。
 * 原固件里遍布 millis() / delay() / delayMicroseconds()，这里集中提供，
 * 避免每个模块各写一遍。
 *********************************************************************/

#ifndef SQUID_SYS_H
#define SQUID_SYS_H

#include <cstdint>
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 上电至今的毫秒数（与 Arduino millis() 语义一致，32 位溢出行为相同）。
inline uint32_t millis() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// 上电至今的微秒数。
inline uint64_t micros64() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

// 毫秒级阻塞延时（让出 CPU 给其它任务/看门狗）。
inline void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// 微秒级忙等（不让出 CPU，用于驱动时序）。
inline void delayMicroseconds(uint32_t us) {
    esp_rom_delay_us(us);
}

// 让出一次 CPU（替代 Arduino yield()）。
inline void yieldCpu() {
    vTaskDelay(1);
}

#endif  // SQUID_SYS_H
