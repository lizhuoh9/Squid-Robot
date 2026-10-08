/**********************************************************************
 * StatusDisplay.h
 *
 * Minima 状态回传帧的解析和显示（读 UART1）。
 *********************************************************************/

#ifndef SQUID_STATUS_DISPLAY_H
#define SQUID_STATUS_DISPLAY_H

#include <cstdint>

#include "Board.h"

class StatusDisplay {
public:
    StatusDisplay();

    void processMinimaFeedback();

    uint8_t getLastMotionStatus() const;
    bool hasRecentHeartbeat(uint32_t nowMs, uint32_t timeoutMs = 3000) const;

    // ── 链路诊断（需配套诊断版 Minima 固件；旧固件不带这些字段，自动忽略）──
    // 取出一份"Minima 实际执行的掩码"新样本；没有新样本返回 false。
    bool takeMinimaMaskSample(uint16_t& mask, uint32_t& rxMs);
    uint32_t getBadFrameCount() const { return _badFrames; }          // ESP32 收到的坏帧
    uint16_t getMinimaBadFrames() const { return _minimaBadFrames; }  // Minima 丢弃的坏帧（累计）
    uint8_t  getMinimaLoopMaxMs() const { return _minimaLoopMaxMs; }  // Minima 上一秒最长 loop
    bool     hasMinimaDiag() const { return _minimaDiag; }
    uint16_t getMinimaMask() const { return _minimaMask; }
    // 执行器回传的时序参数镜像（EEPROM 里的真值）。
    uint16_t getParam(uint8_t id) const { return id < 16 ? _params[id] : 0; }
    bool     hasParams() const { return _paramsSeen; }

private:
    unsigned long _lastHeartbeat;
    uint8_t _lastMotionStatus;
    uint8_t _rxBuffer[FRAME_LENGTH];
    uint8_t _rxIndex;

    uint32_t _badFrames;
    bool     _minimaDiag;
    bool     _maskSamplePending;
    uint16_t _minimaMask;
    uint32_t _minimaMaskRxMs;
    uint16_t _minimaBadFrames;
    uint8_t  _minimaLoopMaxMs;
    uint16_t _params[16] = {};
    bool     _paramsSeen = false;

    void processMotionStatus(uint8_t status);
    void processHeartbeat();
};

#endif  // SQUID_STATUS_DISPLAY_H
