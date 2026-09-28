/**********************************************************************
 * SDLogger.cpp
 *
 * SD 卡 session 日志（POSIX 文件 API over FATFS）。
 *********************************************************************/

#include "SDLogger.h"

#include <cmath>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>

#include "Output.h"
#include "SdCard.h"
#include "Sys.h"

namespace {
// sensors.csv 保持打开，每 20 行（1s @20Hz）flush 一次。
constexpr uint8_t FLUSH_EVERY_N_ROWS = 20;

// fflush 只把 newlib 缓冲写进 FATFS；目录项里的文件大小要 f_sync/close 才更新。
// 不 fsync 的话中途断电，sensors.csv 在卡上是 0 字节、整段数据丢失
// （Arduino 版 File.flush() 自带 f_sync，移植时漏了）。
void flushAndSync(FILE* f) {
    if (!f) return;
    fflush(f);
    fsync(fileno(f));
}

bool fsExists(const char* logical) {
    char full[80];
    sdFsPath(full, sizeof(full), logical);
    struct stat st;
    return stat(full, &st) == 0;
}

long fsSize(const char* logical) {
    char full[80];
    sdFsPath(full, sizeof(full), logical);
    struct stat st;
    return stat(full, &st) == 0 ? static_cast<long>(st.st_size) : -1;
}
}  // namespace

SDLogger::SDLogger()
    : _sdReady(false), _sessionActive(false), _sensorFile(nullptr), _rowsSinceFlush(0) {
    _folder[0] = '\0';
    _sessionId[0] = '\0';
}

bool SDLogger::begin() {
    _sdReady = false;
    if (!sdCardMount()) {
        g_dbg->println("[SDLogger] Mount failed. Check wiring/FAT32.");
        return false;
    }
    const uint32_t mb = static_cast<uint32_t>(sdCardSizeBytes() / (1024ULL * 1024ULL));
    g_dbg->print("[SDLogger] SD ready, ");
    g_dbg->print(mb);
    g_dbg->println(" MB");
    _sdReady = true;
    return true;
}

void SDLogger::diagnose() {
    if (_sessionActive) {
        g_dbg->println("[SD诊断] TEST 记录进行中，先 md 结束会话再诊断。");
        return;
    }
    _sdReady = sdCardDiagnose();
    g_dbg->println(_sdReady ? "[SD] 已就绪。" : "[SD] 仍不可用。");
}

bool SDLogger::isReady()   const { return _sdReady; }
bool SDLogger::hasSession() const { return _sessionActive; }
const char* SDLogger::getFolder() const { return _folder; }

char* SDLogger::buildPath(char* dst, size_t dstLen, const char* leaf) const {
    snprintf(dst, dstLen, "%s/%s", _folder, leaf);
    return dst;
}

bool SDLogger::startSession(const char* folderName) {
    if (!_sdReady) return false;
    if (_sessionActive) endSession(millis());

    // 防止文件夹名碰撞导致覆盖历史 session：已存在则追加 _1/_2...
    char base[32];
    snprintf(base, sizeof(base), "/%s", folderName);
    strncpy(_folder, base, sizeof(_folder));
    _folder[sizeof(_folder) - 1] = '\0';
    for (uint16_t suffix = 1; suffix < 1000 && fsExists(_folder); ++suffix) {
        snprintf(_folder, sizeof(_folder), "%s_%u", base, suffix);
    }
    if (fsExists(_folder)) {
        g_dbg->println("[SDLogger] 文件夹名冲突上限，放弃 session");
        return false;
    }

    char full[80];
    sdFsPath(full, sizeof(full), _folder);
    if (mkdir(full, 0777) != 0) {
        g_dbg->print("[SDLogger] mkdir 失败: ");
        g_dbg->println(_folder);
        return false;
    }

    // session_id = 文件夹名去掉前导 '/'
    strncpy(_sessionId, _folder[0] == '/' ? _folder + 1 : _folder, sizeof(_sessionId));
    _sessionId[sizeof(_sessionId) - 1] = '\0';

    char logical[52];
    char path[80];

    // sensors.csv —— 保持打开的高频写入文件
    buildPath(logical, sizeof(logical), "sensors.csv");
    sdFsPath(path, sizeof(path), logical);
    _sensorFile = fopen(path, "w");
    if (!_sensorFile) {
        g_dbg->println("[SDLogger] Cannot create sensors.csv");
        return false;
    }
    fprintf(_sensorFile,
            "session_id,timestamp_ms,dt_ms,robot_mode,control_mode,depth_valid,imu_valid,"
            "battery_v,target_depth_cm,filtered_depth_cm,depth_speed_cm_s,depth_accel_cm_s2,"
            "roll_deg,pitch_deg,yaw_deg,gyro_x_deg_s,gyro_y_deg_s,gyro_z_deg_s,"
            "front_distance_cm,left_distance_cm,right_distance_cm,depth_err_cm,"
            "u_base,u_residual,u_total,buoyancy_dir_applied,buoyancy_pwm_applied,"
            "balancing,emergency_stop\r\n");
    flushAndSync(_sensorFile);
    _rowsSinceFlush = 0;

    // commands.csv / events.log —— 低频，每次 open/close
    auto writeHeader = [&](const char* leaf, const char* header) {
        buildPath(logical, sizeof(logical), leaf);
        sdFsPath(path, sizeof(path), logical);
        FILE* f = fopen(path, "w");
        if (f) { fprintf(f, "%s\r\n", header); fclose(f); }
    };
    writeHeader("commands.csv", "millis,source,command");
    writeHeader("events.log",   "millis,event");

    _sessionActive = true;
    g_dbg->print("[SDLogger] Session: ");
    g_dbg->println(_folder);
    return true;
}

void SDLogger::endSession(uint32_t ms) {
    if (!_sessionActive) return;
    logEvent(ms, "session_end");
    if (_sensorFile) {
        fflush(_sensorFile);
        fclose(_sensorFile);
        _sensorFile = nullptr;
    }
    _sessionActive = false;
    _rowsSinceFlush = 0;
    g_dbg->println("[SDLogger] Session closed.");
}

void SDLogger::logSensor(const SensorLogRow& r) {
    if (!_sessionActive || !_sensorFile) return;

    // 无效值兜底为 0，避免写出 "nan"。
    auto val = [](float v) { return std::isnan(v) ? 0.0f : v; };

    fprintf(_sensorFile,
            "%s,%u,%u,%u,%u,%d,%d,"
            "%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.1f,%.1f,%.1f,%.2f,"
            "%.3f,%.3f,%.3f,%u,%u,%d,%d\r\n",
            _sessionId, static_cast<unsigned>(r.timestampMs), static_cast<unsigned>(r.dtMs),
            r.robotMode, r.controlMode,
            r.depthValid ? 1 : 0, r.imuValid ? 1 : 0,
            static_cast<double>(val(r.batteryV)), static_cast<double>(val(r.targetDepthCm)),
            static_cast<double>(val(r.filteredDepthCm)), static_cast<double>(val(r.depthSpeedCmS)),
            static_cast<double>(val(r.depthAccelCmS2)),
            static_cast<double>(val(r.roll)), static_cast<double>(val(r.pitch)),
            static_cast<double>(val(r.yaw)), static_cast<double>(val(r.gyroX)),
            static_cast<double>(val(r.gyroY)), static_cast<double>(val(r.gyroZ)),
            static_cast<double>(val(r.frontCm)), static_cast<double>(val(r.leftCm)),
            static_cast<double>(val(r.rightCm)), static_cast<double>(val(r.depthErrCm)),
            static_cast<double>(val(r.uBase)), static_cast<double>(val(r.uResidual)),
            static_cast<double>(val(r.uTotal)),
            r.buoyancyDir, r.buoyancyPwm, r.balancing ? 1 : 0, r.emergencyStop ? 1 : 0);

    if (++_rowsSinceFlush >= FLUSH_EVERY_N_ROWS) {
        flushAndSync(_sensorFile);   // 每秒落盘一次：断电最多丢 1s
        _rowsSinceFlush = 0;
    }
}

void SDLogger::logCommand(uint32_t ms, const char* source, const char* cmd) {
    if (!_sessionActive) return;

    char logical[52];
    char path[80];
    buildPath(logical, sizeof(logical), "commands.csv");
    sdFsPath(path, sizeof(path), logical);
    FILE* f = fopen(path, "a");
    if (!f) return;

    fprintf(f, "%u,%s,%s\r\n", static_cast<unsigned>(ms), source, cmd);
    fclose(f);
}

void SDLogger::logEvent(uint32_t ms, const char* msg) {
    if (!_sessionActive) return;

    char logical[52];
    char path[80];
    buildPath(logical, sizeof(logical), "events.log");
    sdFsPath(path, sizeof(path), logical);
    FILE* f = fopen(path, "a");
    if (!f) return;

    fprintf(f, "%u,%s\r\n", static_cast<unsigned>(ms), msg);
    fclose(f);
}

void SDLogger::printStats() {
    if (!_sdReady) {
        g_dbg->println("[SD] 未就绪");
        return;
    }
    if (!_sessionActive) {
        g_dbg->println("[SD] 无活跃 session（需先进入 MT 模式）");
        return;
    }
    g_dbg->print("[SD] session: ");
    g_dbg->println(_folder);

    // sensors.csv 保持打开，先 flush 再取大小。
    g_dbg->print("  sensors.csv: ");
    if (_sensorFile) {
        flushAndSync(_sensorFile);
        char logical[52];
        buildPath(logical, sizeof(logical), "sensors.csv");
        long sz = fsSize(logical);
        if (sz >= 0) { g_dbg->print(static_cast<long>(sz)); g_dbg->println(" 字节"); }
        else         { g_dbg->println("无法打开"); }
    } else {
        g_dbg->println("无法打开");
    }

    const char* files[] = {"commands.csv", "events.log"};
    char logical[52];
    for (uint8_t i = 0; i < 2; i++) {
        buildPath(logical, sizeof(logical), files[i]);
        long sz = fsSize(logical);
        g_dbg->print("  ");
        g_dbg->print(files[i]);
        g_dbg->print(": ");
        if (sz >= 0) { g_dbg->print(sz); g_dbg->println(" 字节"); }
        else         { g_dbg->println("无法打开"); }
    }
    const uint32_t mb = static_cast<uint32_t>(sdCardSizeBytes() / (1024ULL * 1024ULL));
    g_dbg->print("  SD 卡容量: ");
    g_dbg->print(mb);
    g_dbg->println(" MB");
}
