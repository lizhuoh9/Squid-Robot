/**********************************************************************
 * ImuManager.cpp
 *
 * EBIMU-9DOFV6 解析实现。
 *********************************************************************/

#include "ImuManager.h"

#include <cstdlib>

#include "Board.h"
#include "Output.h"
#include "Sys.h"

namespace {
constexpr uint32_t FRAME_TIMEOUT_MS = 500;   // 500ms 无新帧视为 stale
constexpr size_t   IMU_MAX_BYTES_PER_UPDATE = 64;
constexpr uint32_t CONFIG_CMD_GAP_MS = 100;  // 命令之间的间隔，等 <ok> 回复
}  // namespace

ImuManager::ImuManager(CH9434A* ch9434)
    : _ch9434(ch9434),
      _lineLen(0),
      _roll(0.0f),
      _pitch(0.0f),
      _yaw(0.0f),
      _gyroX(0.0f),
      _gyroY(0.0f),
      _gyroZ(0.0f),
      _valid(false),
      _lastFrameMs(0),
      _sampleCount(0),
      _parseErrorCount(0),
      _debugStatus(DEBUG_NOT_STARTED) {
    _lineBuf[0] = '\0';
}

bool ImuManager::begin() {
    if (!_ch9434) return false;

    if (!_ch9434->config(IMU_UART, IMU_BAUDRATE, CH9434A_LCR_8N1)) {
        return false;
    }
    _ch9434->flush(IMU_UART);

    sendConfigSequence();

    _ch9434->flush(IMU_UART);   // 清掉配置期间 IMU 回复的 <ok> 字节
    _lineLen = 0;
    _debugStatus = DEBUG_WAITING_FIRST_FRAME;
    return true;
}

void ImuManager::sendCommand(const char* cmd) {
    _ch9434->print(IMU_UART, cmd);
    delay(CONFIG_CMD_GAP_MS);
}

void ImuManager::sendConfigSequence() {
    sendCommand("<stop>");
    sendCommand("<sof1>");   // Euler 角输出
    sendCommand("<sog1>");   // 开启陀螺角速度输出
    sendCommand("<soa0>");
    sendCommand("<som0>");
    sendCommand("<sod0>");
    sendCommand("<sot0>");
    sendCommand("<sots0>");
    sendCommand("<sor20>");   // 20ms = 50Hz
    sendCommand("<pons1>");   // 上电自动启动
    sendCommand("<start>");
}

void ImuManager::update() {
    if (!_ch9434) return;

    // 每轮最多读 IMU_MAX_BYTES_PER_UPDATE 字节：CH9434A 每读一字节要 2 次 SPI
    // 寄存器访问，积压（如开机 WiFi 等待 10s 后）一次读完会卡主循环几百毫秒。
    // IMU 约 4B/ms，主循环 ~1kHz，每轮 64B 足够跟上，积压会在几十轮内追平。
    uint8_t chunk[IMU_MAX_BYTES_PER_UPDATE];
    const size_t got = _ch9434->read(IMU_UART, chunk, sizeof(chunk));
    for (size_t k = 0; k < got; ++k) {
        const uint8_t c = chunk[k];

        if (c == '\r') continue;
        if (c == '\n') {
            _lineBuf[_lineLen] = '\0';
            if (_lineLen > 0) {
                if (parseLine(_lineBuf)) {
                    _lastFrameMs = millis();
                    _valid = true;
                    _sampleCount++;
                    _debugStatus = DEBUG_VALID;
                } else {
                    _parseErrorCount++;
                    _debugStatus = DEBUG_PARSE_ERROR;
                }
            }
            _lineLen = 0;
            continue;
        }

        if (_lineLen < sizeof(_lineBuf) - 1) {
            _lineBuf[_lineLen++] = static_cast<char>(c);
        } else {
            _lineLen = 0;
            _parseErrorCount++;
        }
    }

    const uint32_t now = millis();
    if (_valid && (now - _lastFrameMs) > FRAME_TIMEOUT_MS) {
        _valid = false;
        _debugStatus = DEBUG_STALE;
    }
}

bool ImuManager::parseLine(const char* line) {
    if (line[0] != '*') return false;

    const char* p = line + 1;

    char* endptr = nullptr;
    float r = strtof(p, &endptr);
    if (endptr == p || *endptr != ',') return false;
    p = endptr + 1;

    float pi = strtof(p, &endptr);
    if (endptr == p || *endptr != ',') return false;
    p = endptr + 1;

    float y = strtof(p, &endptr);
    if (endptr == p) return false;

    if (r < -180.0f || r > 180.0f) return false;
    if (pi < -90.0f || pi > 90.0f) return false;
    if (y < -180.0f || y > 180.0f) return false;

    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    if (*endptr == ',') {
        p = endptr + 1; gx = strtof(p, &endptr);
        if (*endptr == ',') {
            p = endptr + 1; gy = strtof(p, &endptr);
            if (*endptr == ',') { p = endptr + 1; gz = strtof(p, &endptr); }
        }
    }

    _roll  = r;
    _pitch = pi;
    _yaw   = y;
    _gyroX = gx;
    _gyroY = gy;
    _gyroZ = gz;
    return true;
}

const char* ImuManager::getStatusText() const {
    switch (_debugStatus) {
        case DEBUG_NOT_STARTED:         return "not_started";
        case DEBUG_WAITING_FIRST_FRAME: return "waiting";
        case DEBUG_VALID:               return "valid";
        case DEBUG_STALE:               return "stale";
        case DEBUG_PARSE_ERROR:         return "parse_error";
    }
    return "unknown";
}

void ImuManager::printDebug() const {
    g_dbg->printf("IMU UART3: samples=%u | parse_err=%u | status=%s\r\n",
                  static_cast<unsigned>(_sampleCount),
                  static_cast<unsigned>(_parseErrorCount), getStatusText());
    g_dbg->printf("  roll=%.2f pitch=%.2f yaw=%.2f deg\r\n",
                  static_cast<double>(_roll), static_cast<double>(_pitch),
                  static_cast<double>(_yaw));
}
