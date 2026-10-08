/**********************************************************************
 * DepthSensorManager.cpp
 *
 * MS5837-02BA I2C 读取与深度滤波实现（原生 ESP-IDF I2C）。
 *********************************************************************/

#include "DepthSensorManager.h"

#include <cmath>
#include <cstring>

#include "Board.h"
#include "Output.h"
#include "Sys.h"
#include "driver/i2c.h"

namespace {
constexpr i2c_port_t DEPTH_I2C_PORT = I2C_NUM_0;
constexpr TickType_t I2C_OP_TICKS   = pdMS_TO_TICKS(50);

constexpr uint8_t MS5837_CMD_RESET = 0x1E;
constexpr uint8_t MS5837_CMD_ADC_READ = 0x00;
constexpr uint8_t MS5837_CMD_CONVERT_D1_8192 = 0x4A;
constexpr uint8_t MS5837_CMD_CONVERT_D2_8192 = 0x5A;

constexpr uint32_t MS5837_RESET_DELAY_MS = 20;
constexpr uint32_t MS5837_CONVERSION_DELAY_MS = 40;
constexpr uint32_t SAMPLE_INTERVAL_MS = 50;
constexpr uint32_t DATA_TIMEOUT_MS = 2000;
constexpr uint8_t PROM_WORD_COUNT = 7;
constexpr uint8_t PROM_READ_RETRIES = 3;
constexpr uint8_t ADC_READ_RETRIES = 3;

constexpr float FRESH_WATER_DENSITY_KG_M3 = 997.0f;
constexpr float GRAVITY_M_S2 = 9.80665f;
constexpr float CM_PER_MBAR = 10000.0f / (FRESH_WATER_DENSITY_KG_M3 * GRAVITY_M_S2);

bool g_i2cReady = false;

// 初始化 I2C 主机（幂等）。
bool i2cInit() {
    if (g_i2cReady) return true;
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = DEPTH_I2C_SDA;
    conf.scl_io_num = DEPTH_I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = DEPTH_I2C_FREQ;
    if (i2c_param_config(DEPTH_I2C_PORT, &conf) != ESP_OK) return false;
    if (i2c_driver_install(DEPTH_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) return false;
    g_i2cReady = true;
    return true;
}

esp_err_t i2cWrite(uint8_t addr, const uint8_t* data, size_t len) {
    return i2c_master_write_to_device(DEPTH_I2C_PORT, addr, data, len, I2C_OP_TICKS);
}

esp_err_t i2cRead(uint8_t addr, uint8_t* buf, size_t len) {
    return i2c_master_read_from_device(DEPTH_I2C_PORT, addr, buf, len, I2C_OP_TICKS);
}

bool i2cProbe(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(DEPTH_I2C_PORT, cmd, I2C_OP_TICKS);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}
}  // namespace

DepthSensorManager::DepthSensorManager()
    : _rawDepthCm(0.0f),
      _filteredDepthCm(0.0f),
      _temperatureC(0.0f),
      _pressureMbar(0.0f),
      _zeroPressureMbar(0.0f),
      _depthSpeedCmS(0.0f),
      _depthAccelCmS2(0.0f),
      _valid(false),
      _sensorReady(false),
      _filterInitialized(false),
      _zeroReferenceValid(false),
      _promCrcValid(false),
      _lastUpdate(0),
      _lastFilterUpdate(0),
      _lastSampleMs(0),
      _sampleCount(0),
      _readErrorCount(0),
      _lastD1(0),
      _lastD2(0),
      _lastPromReadIndex(0xFF),
      _lastI2cError(0),
      _lastReadFailure(READ_OK),
      _debugStatus(DEBUG_NOT_STARTED),
      _convPhase(CONV_IDLE),
      _convStartMs(0),
      _convAttempts(0),
      _d1raw(0) {
    for (uint8_t i = 0; i < 8; ++i) {
        _prom[i] = 0;
    }
}

bool DepthSensorManager::begin() {
    _valid = false;
    _sensorReady = false;
    _filterInitialized = false;
    _zeroReferenceValid = false;
    _promCrcValid = false;
    _lastUpdate = 0;
    _lastFilterUpdate = 0;
    _lastSampleMs = 0;
    _sampleCount = 0;
    _readErrorCount = 0;
    _lastD1 = 0;
    _lastD2 = 0;
    _lastPromReadIndex = 0xFF;
    _lastI2cError = 0;
    _lastReadFailure = READ_OK;
    _convPhase = CONV_IDLE;
    _convStartMs = 0;
    _convAttempts = 0;
    _d1raw = 0;

    if (!i2cInit()) {
        _debugStatus = DEBUG_I2C_INIT_FAILED;
        return false;
    }

    if (!resetSensor()) {
        _debugStatus = DEBUG_RESET_FAILED;
        return false;
    }

    if (!readProm()) {
        return false;
    }

    _sensorReady = true;
    _debugStatus = DEBUG_VALID_DATA;
    return true;
}

void DepthSensorManager::update() {
    // 非阻塞采样状态机：不再用 delay(40) 等转换，改成"触发→隔 40ms 下轮再取"，
    // 每次 update() 立即返回，主循环不被卡住。一次完整采样(D1+D2)约 80ms，
    // 但期间控制/超声/IMU 照常跑。
    const uint32_t now = millis();

    if (_sensorReady) {
        switch (_convPhase) {
            case CONV_IDLE:
                // 按采样间隔触发 D1 转换。
                if (_lastSampleMs == 0 || now - _lastSampleMs >= SAMPLE_INTERVAL_MS) {
                    _lastSampleMs = now;
                    if (triggerConversion(MS5837_CMD_CONVERT_D1_8192, READ_D1_TRIGGER_FAILED)) {
                        _convStartMs = now;
                        _convAttempts = 0;
                        _convPhase = CONV_D1_WAIT;
                    } else {
                        ++_readErrorCount;
                    }
                }
                break;

            case CONV_D1_WAIT:
                if (now - _convStartMs >= MS5837_CONVERSION_DELAY_MS) {
                    uint32_t v = 0;
                    if (!fetchAdc(v, READ_D1_FETCH_FAILED)) {
                        ++_readErrorCount;
                        _convPhase = CONV_IDLE;
                    } else if (v == 0) {
                        // ADC 返回 0 = 还没转换完，再等一轮
                        if (++_convAttempts < ADC_READ_RETRIES) {
                            _convStartMs = now;
                        } else {
                            ++_readErrorCount;
                            _convPhase = CONV_IDLE;
                        }
                    } else {
                        _d1raw = v;
                        _lastD1 = v;
                        if (triggerConversion(MS5837_CMD_CONVERT_D2_8192, READ_D2_TRIGGER_FAILED)) {
                            _convStartMs = now;
                            _convAttempts = 0;
                            _convPhase = CONV_D2_WAIT;
                        } else {
                            ++_readErrorCount;
                            _convPhase = CONV_IDLE;
                        }
                    }
                }
                break;

            case CONV_D2_WAIT:
                if (now - _convStartMs >= MS5837_CONVERSION_DELAY_MS) {
                    uint32_t v = 0;
                    if (!fetchAdc(v, READ_D2_FETCH_FAILED)) {
                        ++_readErrorCount;
                        _convPhase = CONV_IDLE;
                    } else if (v == 0) {
                        if (++_convAttempts < ADC_READ_RETRIES) {
                            _convStartMs = now;
                        } else {
                            ++_readErrorCount;
                            _convPhase = CONV_IDLE;
                        }
                    } else {
                        _lastD2 = v;
                        float pressureMbar = 0.0f;
                        float temperatureC = 0.0f;
                        if (computeFromRaw(_d1raw, v, pressureMbar, temperatureC)) {
                            if (!_zeroReferenceValid) {
                                _zeroPressureMbar = pressureMbar;
                                _zeroReferenceValid = true;
                            }
                            commitDepth(pressureToDepthCm(pressureMbar - _zeroPressureMbar),
                                        temperatureC, pressureMbar);
                            _debugStatus = DEBUG_VALID_DATA;
                            ++_sampleCount;
                        } else {
                            ++_readErrorCount;
                        }
                        _convPhase = CONV_IDLE;
                    }
                }
                break;
        }
    }

    // 数据超时判 stale（用最新 millis() 避免下溢）。
    const uint32_t nowAfter = millis();
    if (_valid && nowAfter - _lastUpdate > DATA_TIMEOUT_MS) {
        _valid = false;
        _filterInitialized = false;
        _depthSpeedCmS = 0.0f;
        _depthAccelCmS2 = 0.0f;
        _debugStatus = DEBUG_STALE;
    }
}

void DepthSensorManager::calibrateZero() {
    if (!_zeroReferenceValid) {
        return;
    }

    _zeroPressureMbar = _pressureMbar;
    _rawDepthCm = 0.0f;
    _filteredDepthCm = 0.0f;
    _depthSpeedCmS = 0.0f;
    _depthAccelCmS2 = 0.0f;
    _filterInitialized = false;
    _depthFilter.reset();
}

bool DepthSensorManager::isValid() const { return _valid; }
float DepthSensorManager::getDepthCm() const { return _filteredDepthCm; }
float DepthSensorManager::getRawDepthCm() const { return _rawDepthCm; }
float DepthSensorManager::getDepthSpeedCmS() const { return _depthSpeedCmS; }
float DepthSensorManager::getDepthAccelCmS2() const { return _depthAccelCmS2; }
float DepthSensorManager::getTemperatureC() const { return _temperatureC; }
uint32_t DepthSensorManager::getLastUpdate() const { return _lastUpdate; }

const char* DepthSensorManager::getStatusText() const {
    return debugStatusString(_debugStatus);
}

const char* DepthSensorManager::getFailureText() const {
    return readFailureString(_lastReadFailure);
}

void DepthSensorManager::printDebug() const {
    g_usb->printf("Depth I2C0: addr=0x%02X | samples=%u | err=%u | prom_crc=%s | prom_idx=",
                  static_cast<unsigned>(DEPTH_I2C_ADDRESS),
                  static_cast<unsigned>(_sampleCount), static_cast<unsigned>(_readErrorCount),
                  _promCrcValid ? "yes" : "no");
    if (_lastPromReadIndex == 0xFF) {
        g_usb->print("--");
    } else {
        g_usb->print(static_cast<unsigned int>(_lastPromReadIndex));
    }
    g_usb->printf(" | i2c_err=%u | fail=%s | status=%s\r\n",
                  static_cast<unsigned>(_lastI2cError), readFailureString(_lastReadFailure),
                  debugStatusString(_debugStatus));

    g_usb->printf("  Last D1/D2: %u / %u\r\n",
                  static_cast<unsigned>(_lastD1), static_cast<unsigned>(_lastD2));
    g_usb->printf("  Pressure: %.2f mbar | Zero: %.2f mbar | Raw depth: %.2f cm | "
                  "Filtered depth: %.2f cm | Speed: %.2f cm/s | Accel: %.2f cm/s^2\r\n",
                  static_cast<double>(_pressureMbar), static_cast<double>(_zeroPressureMbar),
                  static_cast<double>(_rawDepthCm), static_cast<double>(_filteredDepthCm),
                  static_cast<double>(_depthSpeedCmS), static_cast<double>(_depthAccelCmS2));
}

bool DepthSensorManager::resetSensor() {
    if (!writeCommand(MS5837_CMD_RESET)) {
        return false;
    }
    delay(MS5837_RESET_DELAY_MS);
    return true;
}

bool DepthSensorManager::readProm() {
    _prom[7] = 0;

    for (uint8_t index = 0; index < PROM_WORD_COUNT; ++index) {
        _lastPromReadIndex = index;
        bool success = false;

        for (uint8_t attempt = 0; attempt < PROM_READ_RETRIES; ++attempt) {
            const uint8_t reg = static_cast<uint8_t>(0xA0 + index * 2);
            esp_err_t werr = i2cWrite(DEPTH_I2C_ADDRESS, &reg, 1);
            _lastI2cError = (werr == ESP_OK) ? 0 : 0xE1;
            if (werr != ESP_OK) {
                delay(2);
                continue;
            }

            delayMicroseconds(300);
            uint8_t rx[2] = {0, 0};
            esp_err_t rerr = i2cRead(DEPTH_I2C_ADDRESS, rx, 2);
            if (rerr == ESP_OK) {
                _prom[index] = static_cast<uint16_t>(rx[0] << 8) | rx[1];
                _lastI2cError = 0;
                success = true;
                break;
            }
            _lastI2cError = 0xFE;
            delay(2);
        }

        if (!success) {
            _debugStatus = DEBUG_PROM_READ_FAILED;
            return false;
        }
    }

    const uint8_t expectedCrc = static_cast<uint8_t>(_prom[0] >> 12);
    const uint8_t actualCrc = calculatePromCrc(_prom);
    _promCrcValid = expectedCrc == actualCrc;
    if (!_promCrcValid) {
        _debugStatus = DEBUG_PROM_CRC_FAILED;
        return false;
    }
    return true;
}

void DepthSensorManager::printI2cScan() {
    bool foundAny = false;

    g_usb->printf("I2C scan on SDA=IO%d, SCL=IO%d\r\n",
                  static_cast<int>(DEPTH_I2C_SDA), static_cast<int>(DEPTH_I2C_SCL));

    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2cProbe(addr)) {
            foundAny = true;
            g_usb->printf("  Found I2C device at 0x%02X\r\n", addr);
        }
    }

    if (!foundAny) {
        g_usb->println("  No I2C devices found.");
    }

    for (uint8_t candidate = 0x76; candidate <= 0x77; ++candidate) {
        g_usb->printf("  Probe 0x%X -> %s\r\n", candidate, i2cProbe(candidate) ? "ACK" : "no ACK");
    }
}

uint8_t DepthSensorManager::calculatePromCrc(const uint16_t prom[8]) const {
    uint16_t promCopy[8];
    for (uint8_t i = 0; i < 8; ++i) {
        promCopy[i] = prom[i];
    }

    uint16_t remainder = 0;
    promCopy[0] &= 0x0FFF;
    promCopy[7] = 0;

    for (uint8_t count = 0; count < 16; ++count) {
        if (count & 1U) {
            remainder ^= static_cast<uint16_t>(promCopy[count >> 1] & 0x00FF);
        } else {
            remainder ^= static_cast<uint16_t>(promCopy[count >> 1] >> 8);
        }

        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (remainder & 0x8000) {
                remainder = static_cast<uint16_t>((remainder << 1) ^ 0x3000);
            } else {
                remainder <<= 1;
            }
        }
    }

    return static_cast<uint8_t>((remainder >> 12) & 0x0F);
}

bool DepthSensorManager::writeCommand(uint8_t command) {
    esp_err_t err = i2cWrite(DEPTH_I2C_ADDRESS, &command, 1);
    _lastI2cError = (err == ESP_OK) ? 0 : 0xE0;
    return err == ESP_OK;
}

// 非阻塞：触发一次转换（只发命令，不等待）。
bool DepthSensorManager::triggerConversion(uint8_t conversionCommand, ReadFailure triggerFailure) {
    if (!writeCommand(conversionCommand)) {
        _debugStatus = DEBUG_ADC_READ_FAILED;
        _lastReadFailure = triggerFailure;
        return false;
    }
    return true;
}

// 非阻塞：取一次 ADC 结果（调用前必须已等够转换时间）。value==0 表示还没转换完。
bool DepthSensorManager::fetchAdc(uint32_t& value, ReadFailure fetchFailure) {
    const uint8_t reg = MS5837_CMD_ADC_READ;
    esp_err_t werr = i2cWrite(DEPTH_I2C_ADDRESS, &reg, 1);
    _lastI2cError = (werr == ESP_OK) ? 0 : 0xE2;
    if (werr != ESP_OK) {
        _debugStatus = DEBUG_ADC_READ_FAILED;
        _lastReadFailure = fetchFailure;
        return false;
    }

    delayMicroseconds(300);   // ~几百微秒，忙等即可，不阻塞主循环节奏
    uint8_t rx[3] = {0, 0, 0};
    esp_err_t rerr = i2cRead(DEPTH_I2C_ADDRESS, rx, 3);
    if (rerr != ESP_OK) {
        _debugStatus = DEBUG_ADC_READ_FAILED;
        _lastReadFailure = fetchFailure;
        _lastI2cError = 0xFD;
        return false;
    }

    value = static_cast<uint32_t>(rx[0]) << 16;
    value |= static_cast<uint32_t>(rx[1]) << 8;
    value |= static_cast<uint32_t>(rx[2]);
    _lastReadFailure = READ_OK;
    _lastI2cError = 0;
    return true;
}

bool DepthSensorManager::computeFromRaw(uint32_t d1, uint32_t d2,
                                        float& pressureMbar, float& temperatureC) {
    const int32_t dT = static_cast<int32_t>(d2) - (static_cast<int32_t>(_prom[5]) << 8);
    int64_t temp = 2000LL + ((static_cast<int64_t>(dT) * _prom[6]) >> 23);
    int64_t off  = (static_cast<int64_t>(_prom[2]) << 17) +
                   ((static_cast<int64_t>(_prom[4]) * dT) >> 6);
    int64_t sens = (static_cast<int64_t>(_prom[1]) << 16) +
                   ((static_cast<int64_t>(_prom[3]) * dT) >> 7);

    int64_t tempCompensation = 0;
    int64_t offCompensation = 0;
    int64_t sensCompensation = 0;
    if (temp < 2000) {
        const int64_t tempDelta = temp - 2000LL;
        tempCompensation = (11LL * static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) >> 35;
        offCompensation = (31LL * tempDelta * tempDelta) >> 3;
        sensCompensation = (63LL * tempDelta * tempDelta) >> 5;
    }

    temp -= tempCompensation;
    off -= offCompensation;
    sens -= sensCompensation;

    const int64_t pressureRaw = ((((static_cast<int64_t>(d1) * sens) >> 21) - off) >> 15);
    pressureMbar = static_cast<float>(pressureRaw) * 0.01f;
    temperatureC = static_cast<float>(temp) * 0.01f;

    if (pressureMbar < 10.0f || pressureMbar > 2000.0f ||
        temperatureC < -40.0f || temperatureC > 85.0f) {
        _debugStatus = DEBUG_VALUE_OUT_OF_RANGE;
        _lastReadFailure = READ_VALUE_OUT_OF_RANGE;
        return false;
    }

    _lastReadFailure = READ_OK;
    return true;
}

float DepthSensorManager::pressureToDepthCm(float pressureDeltaMbar) const {
    if (pressureDeltaMbar <= 0.0f) {
        return 0.0f;
    }
    return pressureDeltaMbar * CM_PER_MBAR;
}

void DepthSensorManager::commitDepth(float depthCm, float temperatureC, float pressureMbar) {
    const uint32_t now = millis();

    _rawDepthCm = depthCm;
    _pressureMbar = pressureMbar;

    if (!_filterInitialized) {
        _depthFilter.reset(depthCm, 0.0f, 0.0f);
        _filteredDepthCm = depthCm;
        _depthSpeedCmS = 0.0f;
        _depthAccelCmS2 = 0.0f;
        _filterInitialized = true;
    } else {
        float dt = static_cast<float>(now - _lastFilterUpdate) * 0.001f;
        if (dt < 0.01f) {
            dt = 0.01f;
        }
        if (dt > 0.20f) {
            dt = 0.20f;
        }

        _depthFilter.update(depthCm, dt);
        _filteredDepthCm = _depthFilter.getPosition();
        _depthSpeedCmS = _depthFilter.getVelocity();
        _depthAccelCmS2 = _depthFilter.getAcceleration();
    }

    _temperatureC = temperatureC;
    _lastUpdate = now;
    _lastFilterUpdate = now;
    _valid = true;
}

const char* DepthSensorManager::debugStatusString(DebugStatus status) {
    switch (status) {
        case DEBUG_NOT_STARTED:       return "not_started";
        case DEBUG_I2C_INIT_FAILED:   return "i2c_init_failed";
        case DEBUG_RESET_FAILED:      return "reset_failed";
        case DEBUG_PROM_READ_FAILED:  return "prom_read_failed";
        case DEBUG_PROM_CRC_FAILED:   return "prom_crc_failed";
        case DEBUG_ADC_READ_FAILED:   return "adc_read_failed";
        case DEBUG_VALUE_OUT_OF_RANGE:return "value_out_of_range";
        case DEBUG_VALID_DATA:        return "valid";
        case DEBUG_STALE:             return "stale";
        default:                      return "unknown";
    }
}

const char* DepthSensorManager::readFailureString(ReadFailure failure) {
    switch (failure) {
        case READ_OK:                 return "ok";
        case READ_D1_TRIGGER_FAILED:  return "d1_trigger";
        case READ_D1_FETCH_FAILED:    return "d1_fetch";
        case READ_D2_TRIGGER_FAILED:  return "d2_trigger";
        case READ_D2_FETCH_FAILED:    return "d2_fetch";
        case READ_VALUE_OUT_OF_RANGE: return "range";
        default:                      return "unknown";
    }
}
