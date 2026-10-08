/**********************************************************************
 * SDLogger.h
 *
 * 基于 session 的 SD 卡日志记录器（SPI3/HSPI）。原生 ESP-IDF：
 * 通过 SdCard 层挂载 FATFS，使用标准 POSIX 文件 API 写入。
 * 每次进入 TEST 模式以时间戳命名新文件夹，分文件记录：
 *   sensors.csv / commands.csv / events.log
 *********************************************************************/

#ifndef SQUID_SD_LOGGER_H
#define SQUID_SD_LOGGER_H

#include <cstdint>
#include <cstdio>

// sensors.csv 一行的全部字段，对齐 learning 残差训练管线的输入契约。
struct SensorLogRow {
    uint32_t timestampMs;
    uint32_t dtMs;
    uint8_t  robotMode;
    uint8_t  controlMode;
    bool     depthValid;
    bool     imuValid;
    float    batteryV;
    float    targetDepthCm;
    float    filteredDepthCm;
    float    depthSpeedCmS;
    float    depthAccelCmS2;
    float    roll, pitch, yaw;
    float    gyroX, gyroY, gyroZ;
    float    frontCm, leftCm, rightCm;
    float    depthErrCm;
    float    pidRaw;
    float    uBase;
    float    uResidual;
    float    uTotal;
    bool     outputSaturated;
    uint8_t  buoyancyDir;
    uint8_t  buoyancyPwm;
    bool     balancing;
    bool     emergencyStop;
};

class SDLogger {
public:
    SDLogger();

    bool begin();
    // 控制台 "sd"：SD 挂载诊断；成功则 SD 恢复可用（不在 TEST 记录中才允许）。
    void diagnose();
    bool isReady() const;
    bool hasSession() const;

    bool startSession(const char* folderName);
    void endSession(uint32_t ms);

    void logSensor(const SensorLogRow& row);
    void logCommand(uint32_t ms, const char* source, const char* cmd);
    void logEvent(uint32_t ms, const char* msg);

    const char* getFolder() const;
    void printStats();

private:
    // 构造逻辑路径 "/<folder>/<leaf>"（不含挂载点前缀）。
    char* buildPath(char* dst, size_t dstLen, const char* leaf) const;

    bool  _sdReady;
    bool  _sessionActive;
    char  _folder[36];       // "/<YYYY-MM-DD_HH-MM-SS>"（逻辑路径）
    char  _sessionId[36];    // 不含前导斜杠的 session 名
    FILE* _sensorFile;
    uint8_t _rowsSinceFlush;
};

#endif  // SQUID_SD_LOGGER_H
