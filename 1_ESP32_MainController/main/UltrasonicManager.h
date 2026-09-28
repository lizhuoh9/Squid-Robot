/**********************************************************************
 * UltrasonicManager.h
 *
 * 三路超声波传感器管理器：轮询 CH9434A 三个 UART，解析回波距离，
 * 卡尔曼滤波后提供给自动寻路。
 *********************************************************************/

#ifndef SQUID_ULTRASONIC_MANAGER_H
#define SQUID_ULTRASONIC_MANAGER_H

#include <cstdint>

#include "Board.h"
#include "CH9434A.h"
#include "KalmanFilter.h"

struct UltrasonicData {
    uint16_t rawDistanceMm;
    uint16_t filteredDistanceMm;
    bool valid;
    uint32_t lastUpdate;
    uint8_t errorCount;
};

class UltrasonicManager {
public:
    enum DebugStatus : uint8_t {
        DEBUG_NO_DATA = 0,
        DEBUG_SHORT_FRAME,
        DEBUG_BAD_CHECKSUM,
        DEBUG_OUT_OF_RANGE,
        DEBUG_VALID_FRAME
    };

    explicit UltrasonicManager(CH9434A* ch9434);

    bool begin();
    void update();

    uint16_t getDistance(uint8_t sensor) const;
    uint16_t getRawDistance(uint8_t sensor) const;
    bool isValid(uint8_t sensor) const;
    uint32_t getLastUpdate(uint8_t sensor) const;
    void getAllDistances(uint16_t* distances) const;
    void printDebug() const;

    void reset(uint8_t sensor);
    void resetAll();

private:
    CH9434A* _ch9434;
    UltrasonicData _sensors[NUM_ULTRASONIC];
    KalmanFilter _filters[NUM_ULTRASONIC];
    bool _filterInitialized[NUM_ULTRASONIC];
    uint32_t _lastTrigger[NUM_ULTRASONIC];
    uint32_t _lastFilterUpdate[NUM_ULTRASONIC];
    uint32_t _lastScanTime;
    uint8_t _currentSensor;
    bool _waitingForResponse;
    uint32_t _rxByteCount[NUM_ULTRASONIC];
    uint8_t _lastRxBuffer[NUM_ULTRASONIC][10];
    uint8_t _lastRxLength[NUM_ULTRASONIC];
    DebugStatus _lastDebugStatus[NUM_ULTRASONIC];

    bool readSensor(uint8_t sensor);
    void triggerMeasurement(uint8_t sensor);
    bool validateFrame(const uint8_t* buffer) const;
    uint16_t parseDistance(const uint8_t* buffer) const;
    uint8_t getUartChannel(uint8_t sensor) const;
};

#endif  // SQUID_ULTRASONIC_MANAGER_H
