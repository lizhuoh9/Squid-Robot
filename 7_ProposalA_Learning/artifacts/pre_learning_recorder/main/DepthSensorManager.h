/**********************************************************************
 * DepthSensorManager.h
 *
 * MS5837-02BA 深度传感器管理器（原生 ESP-IDF I2C 版）。
 * 通过 I2C 读取气压/温度，换算深度后做卡尔曼滤波。
 *********************************************************************/

#ifndef SQUID_DEPTH_SENSOR_MANAGER_H
#define SQUID_DEPTH_SENSOR_MANAGER_H

#include <cstdint>

#include "KalmanFilter.h"

class DepthSensorManager {
public:
    DepthSensorManager();

    bool begin();
    void update();
    void calibrateZero();

    bool isValid() const;
    float getDepthCm() const;
    float getRawDepthCm() const;
    float getDepthSpeedCmS() const;
    float getDepthAccelCmS2() const;
    float getTemperatureC() const;
    uint32_t getLastUpdate() const;
    const char* getStatusText() const;
    const char* getFailureText() const;
    void printDebug() const;

private:
    enum DebugStatus : uint8_t {
        DEBUG_NOT_STARTED = 0,
        DEBUG_I2C_INIT_FAILED,
        DEBUG_RESET_FAILED,
        DEBUG_PROM_READ_FAILED,
        DEBUG_PROM_CRC_FAILED,
        DEBUG_ADC_READ_FAILED,
        DEBUG_VALUE_OUT_OF_RANGE,
        DEBUG_VALID_DATA,
        DEBUG_STALE
    };

    enum ReadFailure : uint8_t {
        READ_OK = 0,
        READ_D1_TRIGGER_FAILED,
        READ_D1_FETCH_FAILED,
        READ_D2_TRIGGER_FAILED,
        READ_D2_FETCH_FAILED,
        READ_VALUE_OUT_OF_RANGE
    };

    float _rawDepthCm;
    float _filteredDepthCm;
    float _temperatureC;
    float _pressureMbar;
    float _zeroPressureMbar;
    float _depthSpeedCmS;
    float _depthAccelCmS2;
    bool _valid;
    bool _sensorReady;
    bool _filterInitialized;
    bool _zeroReferenceValid;
    bool _promCrcValid;
    uint32_t _lastUpdate;
    uint32_t _lastFilterUpdate;
    uint32_t _lastSampleMs;
    uint32_t _sampleCount;
    uint32_t _readErrorCount;
    uint32_t _lastD1;
    uint32_t _lastD2;
    uint8_t _lastPromReadIndex;
    uint8_t _lastI2cError;
    ReadFailure _lastReadFailure;
    uint16_t _prom[8];
    DebugStatus _debugStatus;

    // ── 非阻塞采样状态机（避免 update() 里 delay(40)×2 卡死主循环）──────
    enum ConvPhase : uint8_t { CONV_IDLE, CONV_D1_WAIT, CONV_D2_WAIT };
    ConvPhase _convPhase;
    uint32_t  _convStartMs;   // 当前转换触发时刻
    uint8_t   _convAttempts;  // 同一相位内"转换未完成(读到0)"的重试计数
    uint32_t  _d1raw;         // 已取回的 D1，等 D2 一起算

    KalmanFilter _depthFilter;

    bool resetSensor();
    bool readProm();
    void printI2cScan();
    uint8_t calculatePromCrc(const uint16_t prom[8]) const;
    bool writeCommand(uint8_t command);
    // 非阻塞：触发一次转换 / 取一次 ADC 结果(不含等待,value==0 表示还没转换完)
    bool triggerConversion(uint8_t conversionCommand, ReadFailure triggerFailure);
    bool fetchAdc(uint32_t& value, ReadFailure fetchFailure);
    // 由 D1/D2 原始值算压力/温度(纯数学 + 量程校验)
    bool computeFromRaw(uint32_t d1, uint32_t d2, float& pressureMbar, float& temperatureC);
    float pressureToDepthCm(float pressureDeltaMbar) const;
    void commitDepth(float depthCm, float temperatureC, float pressureMbar);
    static const char* debugStatusString(DebugStatus status);
    static const char* readFailureString(ReadFailure failure);
};

#endif  // SQUID_DEPTH_SENSOR_MANAGER_H
