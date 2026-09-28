/**********************************************************************
 * MotionParams.cpp
 *
 * 参数表的默认值与 EEPROM 持久化（UNO R4 的 EEPROM 由片内 flash 模拟）。
 *********************************************************************/

#include "MotionParams.h"

#include <EEPROM.h>

MotionParams g_params;

namespace {
constexpr int      EE_ADDR  = 0;
constexpr uint32_t EE_MAGIC = 0x53515031UL;   // "SQP1"

struct Blob {
    uint32_t magic;
    uint16_t v[P_COUNT];
    uint8_t  crc;
};

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}
}  // namespace

void MotionParams::loadDefaults() {
    v[P_FWD_INTERVAL]      = 700;
    v[P_FWD_BAL_DELAY]     = 10;
    v[P_FWD_BAL_TIME]      = 800;
    v[P_FWD_BAL_ALT]       = 200;
    v[P_TURN_DURATION]     = 1000;
    v[P_TURN_BAL_DELAY]    = 10;
    v[P_TURN_BAL_TIME]     = 200;
    v[P_TURN_BAL_ALT]      = 100;
    v[P_GLOBAL_BAL_TIME]   = 5000;
    v[P_GLOBAL_BAL_ALT]    = 500;
    v[P_BUOY_BAL_ALT]      = 500;
    v[P_VALVE_GUARD]       = 200;
    v[P_PWM_PERIOD_100US]  = 200;   // 20ms
    v[P_LINK_TIMEOUT]      = 1000;
}

bool MotionParams::load() {
    Blob b{};
    EEPROM.get(EE_ADDR, b);
    const uint8_t want = crc8(reinterpret_cast<const uint8_t*>(&b), sizeof(Blob) - 1);
    if (b.magic != EE_MAGIC || b.crc != want) {
        loadDefaults();
        return false;
    }
    for (uint8_t i = 0; i < P_COUNT; i++) {
        v[i] = b.v[i];
    }
    return true;
}

void MotionParams::save() const {
    Blob b{};
    b.magic = EE_MAGIC;
    for (uint8_t i = 0; i < P_COUNT; i++) {
        b.v[i] = v[i];
    }
    b.crc = crc8(reinterpret_cast<const uint8_t*>(&b), sizeof(Blob) - 1);
    EEPROM.put(EE_ADDR, b);
}

bool MotionParams::set(uint8_t id, uint16_t value) {
    if (id >= P_COUNT) return false;
    // 防呆：这些参数为 0 会让状态机失去节拍，拒掉。
    const bool nonZero = (id == P_FWD_INTERVAL || id == P_FWD_BAL_ALT ||
                          id == P_TURN_DURATION || id == P_TURN_BAL_ALT ||
                          id == P_GLOBAL_BAL_ALT || id == P_BUOY_BAL_ALT ||
                          id == P_PWM_PERIOD_100US);
    if (nonZero && value == 0) return false;
    v[id] = value;
    return true;
}

void MotionParams::print() const {
    static const char* const names[P_COUNT] = {
        "fwd_interval", "fwd_bal_delay", "fwd_bal_time", "fwd_bal_alt",
        "turn_duration", "turn_bal_delay", "turn_bal_time", "turn_bal_alt",
        "global_bal_time", "global_bal_alt", "buoy_bal_alt", "valve_guard",
        "pwm_period_100us", "link_timeout"
    };
    Serial.println(F("[PARAM] id name value"));
    for (uint8_t i = 0; i < P_COUNT; i++) {
        Serial.print(F("  "));
        Serial.print(i);
        Serial.print(' ');
        Serial.print(names[i]);
        Serial.print(F(" = "));
        Serial.println(v[i]);
    }
}
