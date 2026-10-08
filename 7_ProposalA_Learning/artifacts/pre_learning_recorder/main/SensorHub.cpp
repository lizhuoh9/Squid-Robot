/**********************************************************************
 * SensorHub.cpp
 *
 * 汇总 ESP32 串口控制台上显示的基础传感器输出。
 * 电池电压用 adc_oneshot 采样（原 Arduino analogRead）。
 *********************************************************************/

#include "SensorHub.h"

#include <cstdarg>
#include <cstdio>

#include "Board.h"
#include "Output.h"
#include "Sys.h"
#include "esp_adc/adc_oneshot.h"

namespace {
constexpr uint32_t DISPLAY_INTERVAL_MS = 1000;
// HC-12 只有 9600 波特：整屏面板 323 字节 = 0.34 秒空口。而 HC-12 是半双工——
// 它发的这 0.34 秒里收不到任何东西，遥控命令直接消失在空中。2s 一屏也还有 17%
// 的时间是聋的，实测日志里命令连丢、重发全部撞进同一个发送窗口。
// 所以无线上不再发整屏，只发一行紧凑遥测（~70 字节 = 0.07 秒，占用 <4%）；
// USB 那一路仍是 1s 一整屏，信息量不变。
constexpr uint32_t HC12_DISPLAY_INTERVAL_MS = 2000;
// 电池电压：泵阀一动就压降，瞬时值随运动控制上下跳，读出来比真实电量低一截。
// 所以按 BATT_SAMPLE_MS 密采，对外报 3 秒窗口内的【最大值】—— 那一次是最接近
// 空载的采样，最能代表真实电量。窗口长度 = BATT_SAMPLE_MS × BATT_WINDOW。
constexpr uint32_t BATT_SAMPLE_MS      = 250;   // 采样间隔
// BATT_WINDOW 定义在 Board.h（SensorHub.h 的成员数组要用）
const char* SENSOR_NAMES[NUM_ULTRASONIC] = {"Front", "Left ", "Right"};

// IO3 → ESP32-S3 ADC1_CH2。
constexpr adc_channel_t BATT_ADC_CHANNEL = ADC_CHANNEL_2;

adc_oneshot_unit_handle_t g_adc = nullptr;

void adcInitOnce() {
    if (g_adc) return;
    adc_oneshot_unit_init_cfg_t unitCfg = {};
    unitCfg.unit_id = ADC_UNIT_1;
    if (adc_oneshot_new_unit(&unitCfg, &g_adc) != ESP_OK) return;
    adc_oneshot_chan_cfg_t chanCfg = {};
    chanCfg.atten = ADC_ATTEN_DB_12;   // ~0-3.1V（对应原 analogSetAttenuation(ADC_11db)）
    chanCfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    adc_oneshot_config_channel(g_adc, BATT_ADC_CHANNEL, &chanCfg);
}

void printMotionFlags(uint8_t status) {
    if (status & 0x01) g_dbg->print("FWD ");
    if (status & 0x02) g_dbg->print("TURN ");
    if (status & 0x04) g_dbg->print("BUOY ");
    if (status == 0) g_dbg->print("IDLE");
}

// 往定长缓冲尾部追加格式化文本，越界就截断；返回追加后的长度（始终 < cap）。
// 直接写 n += snprintf(buf + n, cap - n, ...) 在截断时会让 n 超过 cap，下一次
// 调用就越界了，所以统一走这里。
int appendFmt(char* buf, size_t cap, int n, const char* fmt, ...) {
    if (cap == 0) return 0;
    if (n < 0) n = 0;
    if (static_cast<size_t>(n) >= cap - 1) return static_cast<int>(cap) - 1;

    va_list ap;
    va_start(ap, fmt);
    const int w = vsnprintf(buf + n, cap - static_cast<size_t>(n), fmt, ap);
    va_end(ap);
    if (w <= 0) return n;

    const size_t total = static_cast<size_t>(n) + static_cast<size_t>(w);
    return static_cast<int>(total >= cap ? cap - 1 : total);
}
}  // namespace

SensorHub::SensorHub()
    : _depthMgr(nullptr),
      _statusDisplay(nullptr),
      _ultrasonicMgr(nullptr),
      _imuMgr(nullptr),
      _lastDisplay(0),
      _lastHc12Display(0),
      _lastBattMs(0),
      _lastBattV(0.0f) {}

void SensorHub::setDepthSensorManager(DepthSensorManager* manager) { _depthMgr = manager; }
void SensorHub::setStatusDisplay(StatusDisplay* display) { _statusDisplay = display; }
void SensorHub::setUltrasonicManager(UltrasonicManager* manager) { _ultrasonicMgr = manager; }
void SensorHub::setImuManager(ImuManager* manager) { _imuMgr = manager; }

float SensorHub::readBatteryVoltage() const {
    adcInitOnce();
    if (!g_adc) return 0.0f;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < BATT_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(g_adc, BATT_ADC_CHANNEL, &raw);
        sum += static_cast<uint32_t>(raw);
        delayMicroseconds(200);
    }
    float vgpio = static_cast<float>(sum / BATT_SAMPLES) / BATT_ADC_MAX * BATT_ADC_REF_V;
    return vgpio * BATT_DIVIDER_RATIO;
}

void SensorHub::updateBattery() {
    const uint32_t now = millis();
    if (now - _lastBattMs < BATT_SAMPLE_MS) return;
    _lastBattMs = now;

    _battWindow[_battIdx] = readBatteryVoltage();
    _battIdx = static_cast<uint8_t>((_battIdx + 1) % BATT_WINDOW);
    if (_battCount < BATT_WINDOW) ++_battCount;

    // 窗口内取最大：泵阀带载时的低谷不代表电量，峰值才接近空载电压。
    float peak = 0.0f;
    for (uint8_t i = 0; i < _battCount; ++i) {
        if (_battWindow[i] > peak) peak = _battWindow[i];
    }
    _lastBattV = peak;
}

void SensorHub::calibrateDepthZero() {
    if (_depthMgr) {
        _depthMgr->calibrateZero();
    }
}

void SensorHub::displayAll() {
    if (!_displayOn) return;
    const uint32_t now = millis();
    if (now - _lastDisplay < DISPLAY_INTERVAL_MS) {
        return;
    }
    _lastDisplay = now;

    // 整屏面板永远只走 USB —— 无线带不动（见文件头常量处的说明）。
    Output* saved = g_dbg;
    g_dbg = g_usb;
    renderAll();
    g_dbg = saved;

    // 无线：每 HC12_DISPLAY_INTERVAL_MS 发一帧遥测。
    if (now - _lastHc12Display >= HC12_DISPLAY_INTERVAL_MS) {
        _lastHc12Display = now;
        sendTelemetryHc12();
    }
}

bool SensorHub::toggleDisplay() {
    _displayOn = !_displayOn;
    return _displayOn;
}

void SensorHub::renderAll() {
    g_dbg->println("\r\n================ ALL SENSORS ================");

    g_dbg->print("Depth: ");
    if (!_depthMgr) {
        g_dbg->println("disabled");
    } else if (!_depthMgr->isValid()) {
        g_dbg->print("⚠ 传感器故障 (");
        g_dbg->print(_depthMgr->getStatusText());
        g_dbg->println(")");
        _depthMgr->printDebug();
    } else {
        g_dbg->print(_depthMgr->getDepthCm(), 2);
        g_dbg->print(" cm | vz=");
        g_dbg->print(_depthMgr->getDepthSpeedCmS(), 2);
        g_dbg->print(" cm/s | az=");
        g_dbg->print(_depthMgr->getDepthAccelCmS2(), 2);
        g_dbg->println(" cm/s^2");
        _depthMgr->printDebug();
    }

    if (_ultrasonicMgr) {
        for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
            g_dbg->print("Ultrasonic ");
            g_dbg->print(SENSOR_NAMES[sensor]);
            g_dbg->print(": ");

            if (_ultrasonicMgr->isValid(sensor)) {
                g_dbg->print(_ultrasonicMgr->getDistance(sensor) / 10.0f, 1);
                g_dbg->println(" cm");
            } else {
                g_dbg->println("offline");
            }
        }
    }

    if (_imuMgr) {
        g_dbg->print("IMU: ");
        if (_imuMgr->isValid()) {
            g_dbg->print("roll=");  g_dbg->print(_imuMgr->getRoll(),  2);
            g_dbg->print(" pitch="); g_dbg->print(_imuMgr->getPitch(), 2);
            g_dbg->print(" yaw=");   g_dbg->print(_imuMgr->getYaw(),   2);
            g_dbg->println(" deg");
        } else {
            g_dbg->print("⚠ 传感器故障 (");
            g_dbg->print(_imuMgr->getStatusText());
            g_dbg->println(")");
            _imuMgr->printDebug();
        }
    }

    if (_statusDisplay) {
        g_dbg->print("Minima: motion=");
        printMotionFlags(_statusDisplay->getLastMotionStatus());
        g_dbg->println();
    }

    g_dbg->print("Battery: ");
    g_dbg->print(_lastBattV, 2);
    g_dbg->print(" V");
    if      (_lastBattV >= 12.0f) g_dbg->print("  [满电]");
    else if (_lastBattV >= 11.1f) g_dbg->print("  [正常]");
    else if (_lastBattV >= 10.5f) g_dbg->print("  [偏低]");
    else if (_lastBattV >  5.0f)  g_dbg->print("  [⚠ 低电！]");
    else                          g_dbg->print("  [未检测]");
    g_dbg->println();

    g_dbg->println("=============================================\r\n");
}

// 无线遥测帧。HC-12 只有 9600 波特且是半双工——它发的时候收不到东西，所以
// 下行每省一个字节，上行命令就多一分把握。这里按 NMEA 的老规矩来：
//
//   $T,<深度mm>,<vz mm/s>,<az mm/s2>,<前cm>,<左cm>,<右cm>,<roll>,<pitch>,<yaw>,<电池cV>,<运动>*<校验>
//
//   - 全部整数，没有小数点和单位，字段靠位置认；空字段 = 该传感器无效/离线
//   - 角度单位 0.1 度（-1800~1800），电池单位 0.01V（1204 = 12.04V）
//   - 运动是位掩码 0~7：bit0 前进 bit1 转向 bit2 浮沉
//   - *XX 是 $ 与 * 之间所有字节的异或校验，两位十六进制
//
// 典型一帧约 40 字节（原来的键值对格式是 68），2s 一帧 = 占用不到 2%。
// 校验和不是摆设：链路上串了字，控制台能直接丢掉整帧，而不是把乱数字画到仪表盘上。
//
// **必须一次性写入**：Hc12Output 是"发送缓冲装不下就整段丢弃"，拆成多次 print
// 会在缓冲临界时把一帧撕成两半（日志里见过 "[FAILS…" 这种半截行）。
void SensorHub::sendTelemetryHc12() {
    char buf[128];
    int n = appendFmt(buf, sizeof(buf), 0, "$T");

    // 深度三兄弟：cm -> mm，无效时留空字段
    if (_depthMgr && _depthMgr->isValid()) {
        n = appendFmt(buf, sizeof(buf), n, ",%d,%d,%d",
                      static_cast<int>(_depthMgr->getDepthCm() * 10.0f),
                      static_cast<int>(_depthMgr->getDepthSpeedCmS() * 10.0f),
                      static_cast<int>(_depthMgr->getDepthAccelCmS2() * 10.0f));
    } else {
        n = appendFmt(buf, sizeof(buf), n, ",,,");
    }

    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
        if (_ultrasonicMgr && _ultrasonicMgr->isValid(sensor)) {
            n = appendFmt(buf, sizeof(buf), n, ",%d",
                          static_cast<int>(_ultrasonicMgr->getDistance(sensor) / 10));
        } else {
            n = appendFmt(buf, sizeof(buf), n, ",");
        }
    }

    if (_imuMgr && _imuMgr->isValid()) {
        n = appendFmt(buf, sizeof(buf), n, ",%d,%d,%d",
                      static_cast<int>(_imuMgr->getRoll() * 10.0f),
                      static_cast<int>(_imuMgr->getPitch() * 10.0f),
                      static_cast<int>(_imuMgr->getYaw() * 10.0f));
    } else {
        n = appendFmt(buf, sizeof(buf), n, ",,,");
    }

    n = appendFmt(buf, sizeof(buf), n, ",%d,%u",
                  static_cast<int>(_lastBattV * 100.0f),
                  static_cast<unsigned>(_statusDisplay ? _statusDisplay->getLastMotionStatus() : 0));

    // NMEA 校验：$ 与 * 之间所有字节异或
    uint8_t sum = 0;
    for (int k = 1; k < n; ++k) sum ^= static_cast<uint8_t>(buf[k]);
    n = appendFmt(buf, sizeof(buf), n, "*%02X\r\n", static_cast<unsigned>(sum));

    if (n > 0) g_hc12Out.write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n));
}

void SensorHub::forceDisplayAll() {
    _lastDisplay = _lastHc12Display = millis();
    // 整屏只给 USB；无线补一行紧凑遥测，遥控端也能立刻看到一次。
    Output* saved = g_dbg;
    g_dbg = g_usb;
    renderAll();
    g_dbg = saved;
    sendTelemetryHc12();
}

void SensorHub::displayCompact() {
    g_dbg->print("Sensors: depth=");
    if (_depthMgr) {
        g_dbg->print(_depthMgr->getDepthCm(), 1);
        g_dbg->print("cm");
    } else {
        g_dbg->print("--");
    }

    if (_ultrasonicMgr) {
        g_dbg->print(" | us=");
        for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
            if (_ultrasonicMgr->isValid(sensor)) {
                g_dbg->print(_ultrasonicMgr->getDistance(sensor) / 10.0f, 0);
            } else {
                g_dbg->print("--");
            }

            if (sensor + 1 < NUM_ULTRASONIC) {
                g_dbg->print(' ');
            }
        }
    }

    g_dbg->print(" | batt=");
    g_dbg->print(_lastBattV, 1);
    g_dbg->print("V");
    g_dbg->println();
}

bool SensorHub::isHealthy() const {
    const bool depthOk = !_depthMgr || _depthMgr->isValid();
    uint8_t ultrasonicOk = 0;

    if (_ultrasonicMgr) {
        for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
            if (_ultrasonicMgr->isValid(sensor)) {
                ++ultrasonicOk;
            }
        }
    }

    return depthOk && ultrasonicOk >= 2;
}

bool SensorHub::hasDepthSensor() const {
    return _depthMgr != nullptr;
}

bool SensorHub::isDepthOnline() const {
    return _depthMgr && _depthMgr->isValid();
}

uint8_t SensorHub::getSensorCount() const {
    uint8_t count = 0;

    if (_depthMgr && _depthMgr->isValid()) {
        ++count;
    }

    if (_ultrasonicMgr) {
        for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
            if (_ultrasonicMgr->isValid(sensor)) {
                ++count;
            }
        }
    }

    return count;
}
