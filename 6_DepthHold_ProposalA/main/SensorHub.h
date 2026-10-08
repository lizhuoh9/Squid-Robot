/**********************************************************************
 * SensorHub.h
 *
 * 汇总各传感器和 Minima 回传，输出到 ESP32 控制台。
 *********************************************************************/

#ifndef SQUID_SENSOR_HUB_H
#define SQUID_SENSOR_HUB_H

#include <cstdint>

#include "DepthSensorManager.h"
#include "ImuManager.h"
#include "StatusDisplay.h"
#include "UltrasonicManager.h"

class SensorHub {
public:
    SensorHub();

    void setDepthSensorManager(DepthSensorManager* manager);
    void setStatusDisplay(StatusDisplay* display);
    void setUltrasonicManager(UltrasonicManager* manager);
    void setImuManager(ImuManager* manager);

    void calibrateDepthZero();
    void displayAll();
    void forceDisplayAll();
    bool toggleDisplay();
    bool isDisplayOn() const { return _displayOn; }
    void displayCompact();
    bool isHealthy() const;
    bool hasDepthSensor() const;
    bool isDepthOnline() const;
    uint8_t getSensorCount() const;

    float readBatteryVoltage() const;
    // 3 秒窗口内的最大值，不是瞬时值（原因见 SensorHub.cpp 的采样常量处）。
    float getBatteryVoltage() const { return _lastBattV; }
    void  updateBattery();

private:
    void renderAll();
    // 无线专用：整屏面板压成一行遥测帧，格式见 SensorHub.cpp。
    void sendTelemetryHc12();

    DepthSensorManager* _depthMgr;
    StatusDisplay*      _statusDisplay;
    UltrasonicManager*  _ultrasonicMgr;
    ImuManager*         _imuMgr;
    bool     _displayOn = true;
    uint32_t _lastDisplay;
    uint32_t _lastHc12Display;   // HC-12 上一帧面板的时刻（无线降频用）
    uint32_t _lastBattMs;
    float    _lastBattV;          // 窗口最大值，对外就是"电池电压"
    float    _battWindow[BATT_WINDOW] = {};   // 环形缓冲：最近 3 秒的采样
    uint8_t  _battIdx = 0;
    uint8_t  _battCount = 0;      // 开机头 3 秒还没填满时用
};

#endif  // SQUID_SENSOR_HUB_H
