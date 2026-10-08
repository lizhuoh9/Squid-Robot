#pragma once
#include <cstdint>
#include "esp_http_server.h"

// All actuator values are ESP command/estimate values, NOT measured pump flow
// or Minima pin feedback. Battery is SensorHub's 3-second maximum window.
#pragma pack(push, 1)
struct CaptureRecord {
    uint32_t elapsedUs, depthAgeMs, depthSequence;
    float depthCm, rawDepthCm, speedCmS, accelCmS2, targetCm;
    float desiredAccelCmS2, biasCmS2, requestedDuty, batteryWindowV;
    uint16_t valveMaskEstimated;
    uint8_t pwmCommanded, directionCommanded, flags, reserved[3];
};
struct CaptureHeader {
    char magic[8];
    uint16_t version, recordBytes;
    uint32_t count, intervalUs, crc32, flags, missedSlots;
    float kp, ki, kd, sinkGain, riseGain, restAccel, alpha, referenceDepth;
};
#pragma pack(pop)
static_assert(sizeof(CaptureRecord) == 56, "Capture ABI mismatch");
static_assert(sizeof(CaptureHeader) == 64, "Capture header ABI mismatch");
enum CaptureFlags : uint8_t {
    CAP_DEPTH_VALID=1, CAP_DEPTH_FRESH=2, CAP_TARGET_VALID=4,
    CAP_FORWARD=8, CAP_BALANCE=16, CAP_SATURATED=32
};
bool ramCaptureDue(uint64_t nowUs);
// Called on the motion task; the recorder takes its lock non-blockingly.
void ramCapturePush(CaptureRecord record, uint64_t nowUs, uint32_t depthStampMs);
void ramCaptureRegister(httpd_handle_t server);
