/**********************************************************************
 * UltrasonicManager.cpp
 *
 * 三路超声波传感器的轮询、触发、解析和卡尔曼滤波。
 *********************************************************************/

#include "UltrasonicManager.h"

#include <cstring>

#include "Output.h"
#include "Sys.h"

namespace {
constexpr uint32_t SCAN_INTERVAL_MS = 5;
constexpr uint32_t RESPONSE_WAIT_MS = 80;
constexpr uint32_t DATA_TIMEOUT_MS = 2000;

const uint8_t UART_CHANNELS[NUM_ULTRASONIC] = {
    ULTRASONIC_FRONT_UART,
    ULTRASONIC_LEFT_UART,
    ULTRASONIC_RIGHT_UART,
};

const char* debugStatusToString(UltrasonicManager::DebugStatus status) {
    switch (status) {
        case UltrasonicManager::DEBUG_NO_DATA:     return "no_data";
        case UltrasonicManager::DEBUG_SHORT_FRAME: return "short_frame";
        case UltrasonicManager::DEBUG_BAD_CHECKSUM:return "bad_checksum";
        case UltrasonicManager::DEBUG_OUT_OF_RANGE:return "out_of_range";
        case UltrasonicManager::DEBUG_VALID_FRAME: return "valid";
        default:                                   return "unknown";
    }
}
}  // namespace

UltrasonicManager::UltrasonicManager(CH9434A* ch9434)
    : _ch9434(ch9434),
      _lastScanTime(0),
      _currentSensor(0),
      _waitingForResponse(false) {
    resetAll();
}

bool UltrasonicManager::begin() {
    g_usb->println("Initializing ultrasonic sensors...");

    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
        const uint8_t uart = getUartChannel(sensor);
        g_usb->printf("  Configuring UART%u... ", uart);

        if (!_ch9434->config(uart, ULTRASONIC_BAUDRATE, CH9434A_LCR_8N1)) {
            g_usb->println("FAILED");
            return false;
        }

        _ch9434->flush(uart);
        g_usb->println("OK");
    }

    return true;
}

void UltrasonicManager::update() {
    const uint32_t scanNow = millis();
    if (scanNow - _lastScanTime < SCAN_INTERVAL_MS) {
        return;
    }

    _lastScanTime = scanNow;

    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
        if (_sensors[sensor].lastUpdate == 0 || scanNow - _sensors[sensor].lastUpdate > DATA_TIMEOUT_MS) {
            _sensors[sensor].valid = false;
            _filterInitialized[sensor] = false;
        }
    }

    if (!_waitingForResponse) {
        triggerMeasurement(_currentSensor);
        _lastTrigger[_currentSensor] = scanNow;
        _waitingForResponse = true;
        return;
    }

    if (scanNow - _lastTrigger[_currentSensor] < RESPONSE_WAIT_MS) {
        return;
    }

    readSensor(_currentSensor);
    _currentSensor = static_cast<uint8_t>((_currentSensor + 1) % NUM_ULTRASONIC);
    _waitingForResponse = false;
}

uint16_t UltrasonicManager::getDistance(uint8_t sensor) const {
    return sensor < NUM_ULTRASONIC ? _sensors[sensor].filteredDistanceMm : 0;
}

uint16_t UltrasonicManager::getRawDistance(uint8_t sensor) const {
    return sensor < NUM_ULTRASONIC ? _sensors[sensor].rawDistanceMm : 0;
}

bool UltrasonicManager::isValid(uint8_t sensor) const {
    return sensor < NUM_ULTRASONIC && _sensors[sensor].valid;
}

uint32_t UltrasonicManager::getLastUpdate(uint8_t sensor) const {
    return sensor < NUM_ULTRASONIC ? _sensors[sensor].lastUpdate : 0;
}

void UltrasonicManager::getAllDistances(uint16_t* distances) const {
    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
        distances[sensor] = _sensors[sensor].valid ? _sensors[sensor].filteredDistanceMm : 0;
    }
}

void UltrasonicManager::printDebug() const {
    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
        const uint8_t uart = getUartChannel(sensor);
        g_usb->printf("US UART%u (%u): LSR=0x%X | bytes=%u | err=%u | last_len=%u | status=%s\r\n",
                      static_cast<unsigned>(uart), static_cast<unsigned>(sensor),
                      static_cast<unsigned>(_ch9434 ? _ch9434->getLineStatus(uart) : 0),
                      static_cast<unsigned>(_rxByteCount[sensor]),
                      static_cast<unsigned>(_sensors[sensor].errorCount),
                      static_cast<unsigned>(_lastRxLength[sensor]),
                      debugStatusToString(_lastDebugStatus[sensor]));

        g_usb->print("  Last RX : ");
        for (uint8_t i = 0; i < _lastRxLength[sensor]; ++i) {
            g_usb->printf("%02X ", _lastRxBuffer[sensor][i]);
        }
        g_usb->println();
    }
}

void UltrasonicManager::reset(uint8_t sensor) {
    if (sensor >= NUM_ULTRASONIC) {
        return;
    }

    _sensors[sensor].rawDistanceMm = 0;
    _sensors[sensor].filteredDistanceMm = 0;
    _sensors[sensor].valid = false;
    _sensors[sensor].lastUpdate = 0;
    _sensors[sensor].errorCount = 0;
    _filterInitialized[sensor] = false;
    _lastTrigger[sensor] = 0;
    _lastFilterUpdate[sensor] = 0;
    _rxByteCount[sensor] = 0;
    _lastRxLength[sensor] = 0;
    _lastDebugStatus[sensor] = DEBUG_NO_DATA;
    memset(_lastRxBuffer[sensor], 0, sizeof(_lastRxBuffer[sensor]));
    _filters[sensor].reset();
    _ch9434->flush(getUartChannel(sensor));
}

void UltrasonicManager::resetAll() {
    _currentSensor = 0;
    _waitingForResponse = false;
    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; ++sensor) {
        _sensors[sensor].rawDistanceMm = 0;
        _sensors[sensor].filteredDistanceMm = 0;
        _sensors[sensor].valid = false;
        _sensors[sensor].lastUpdate = 0;
        _sensors[sensor].errorCount = 0;
        _filterInitialized[sensor] = false;
        _lastTrigger[sensor] = 0;
        _lastFilterUpdate[sensor] = 0;
        _rxByteCount[sensor] = 0;
        _lastRxLength[sensor] = 0;
        _lastDebugStatus[sensor] = DEBUG_NO_DATA;
        memset(_lastRxBuffer[sensor], 0, sizeof(_lastRxBuffer[sensor]));
        _filters[sensor].reset();
    }
}

bool UltrasonicManager::readSensor(uint8_t sensor) {
    if (sensor >= NUM_ULTRASONIC) {
        return false;
    }

    const uint8_t uart = getUartChannel(sensor);
    const uint32_t now = millis();

    if (_ch9434->available(uart) == 0) {
        _lastDebugStatus[sensor] = DEBUG_NO_DATA;
        if (++_sensors[sensor].errorCount > 3) {
            _sensors[sensor].valid = false;
            _filterInitialized[sensor] = false;
        }
        return false;
    }

    uint8_t buffer[10];
    uint8_t bytesRead = 0;
    const uint32_t start = millis();

    while (bytesRead < sizeof(buffer) && (millis() - start) < ULTRASONIC_TIMEOUT) {
        if (_ch9434->available(uart)) {
            buffer[bytesRead++] = _ch9434->read(uart);
        } else if (bytesRead >= 4) {
            break;
        }
        delayMicroseconds(100);
    }

    _rxByteCount[sensor] += bytesRead;
    _lastRxLength[sensor] = bytesRead;
    if (bytesRead > 0) {
        memcpy(_lastRxBuffer[sensor], buffer, bytesRead);
    }
    for (uint8_t i = bytesRead; i < sizeof(_lastRxBuffer[sensor]); ++i) {
        _lastRxBuffer[sensor][i] = 0;
    }

    if (bytesRead < 4) {
        _lastDebugStatus[sensor] = bytesRead == 0 ? DEBUG_NO_DATA : DEBUG_SHORT_FRAME;
        ++_sensors[sensor].errorCount;
        return false;
    }

    int frameStart = -1;
    for (uint8_t i = 0; i <= bytesRead - 4; ++i) {
        if (buffer[i] == 0xFF) {
            frameStart = i;
            break;
        }
    }

    if (frameStart < 0 || !validateFrame(&buffer[frameStart])) {
        _lastDebugStatus[sensor] = DEBUG_BAD_CHECKSUM;
        ++_sensors[sensor].errorCount;
        return false;
    }

    const uint16_t rawDistanceMm = parseDistance(&buffer[frameStart]);
    if (rawDistanceMm < 50 || rawDistanceMm > 3000) {
        _lastDebugStatus[sensor] = DEBUG_OUT_OF_RANGE;
        ++_sensors[sensor].errorCount;
        return false;
    }

    _lastDebugStatus[sensor] = DEBUG_VALID_FRAME;

    const float rawDistanceCm = rawDistanceMm / 10.0f;
    float filteredDistanceCm = rawDistanceCm;

    if (!_filterInitialized[sensor]) {
        _filters[sensor].reset(rawDistanceCm, 0.0f);
        _filterInitialized[sensor] = true;
    } else {
        float dt = static_cast<float>(now - _lastFilterUpdate[sensor]) * 0.001f;
        if (dt < 0.05f) {
            dt = 0.05f;
        }
        if (dt > 0.20f) {
            dt = 0.20f;
        }

        _filters[sensor].update(rawDistanceCm, dt);
        filteredDistanceCm = _filters[sensor].getPosition();
    }

    _sensors[sensor].rawDistanceMm = rawDistanceMm;
    _sensors[sensor].filteredDistanceMm = static_cast<uint16_t>(filteredDistanceCm * 10.0f);
    _sensors[sensor].valid = true;
    _sensors[sensor].lastUpdate = now;
    _sensors[sensor].errorCount = 0;
    _lastFilterUpdate[sensor] = now;
    return true;
}

void UltrasonicManager::triggerMeasurement(uint8_t sensor) {
    const uint8_t uart = getUartChannel(sensor);
    _ch9434->flush(uart);
    _ch9434->write(uart, 0x00);
}

bool UltrasonicManager::validateFrame(const uint8_t* buffer) const {
    return buffer[0] == 0xFF &&
           static_cast<uint8_t>(buffer[0] + buffer[1] + buffer[2]) == buffer[3];
}

uint16_t UltrasonicManager::parseDistance(const uint8_t* buffer) const {
    return static_cast<uint16_t>(buffer[1] << 8) | buffer[2];
}

uint8_t UltrasonicManager::getUartChannel(uint8_t sensor) const {
    return UART_CHANNELS[sensor];
}
