/**********************************************************************
 * StatusDisplay.cpp
 *
 * Minima 回传状态帧的接收和串口显示。
 *********************************************************************/

#include "StatusDisplay.h"

#include "Output.h"
#include "Sys.h"
#include "driver/uart.h"

StatusDisplay::StatusDisplay()
    : _lastHeartbeat(0),
      _lastMotionStatus(0),
      _rxIndex(0),
      _badFrames(0),
      _minimaDiag(false),
      _maskSamplePending(false),
      _minimaMask(0),
      _minimaMaskRxMs(0),
      _minimaBadFrames(0),
      _minimaLoopMaxMs(0) {}

bool StatusDisplay::takeMinimaMaskSample(uint16_t& mask, uint32_t& rxMs) {
    if (!_maskSamplePending) return false;
    _maskSamplePending = false;
    mask = _minimaMask;
    rxMs = _minimaMaskRxMs;
    return true;
}

void StatusDisplay::processMinimaFeedback() {
    uint8_t byte = 0;
    while (uart_read_bytes(UART_TO_MINIMA_PORT, &byte, 1, 0) == 1) {
        if (_rxIndex == 0 && byte != FRAME_HEADER) {
            continue;
        }

        _rxBuffer[_rxIndex++] = byte;
        if (_rxIndex < FRAME_LENGTH) {
            continue;
        }

        const bool validLength = _rxBuffer[1] == FRAME_LENGTH;
        const bool validTail = _rxBuffer[7] == FRAME_TAIL;
        const uint8_t crc = calculateCRC8(&_rxBuffer[2], 4);
        const bool validCrc = crc == _rxBuffer[6];
        _rxIndex = 0;

        if (!validLength || !validTail || !validCrc) {
            ++_badFrames;
            continue;
        }

        switch (_rxBuffer[2]) {
            case STATUS_MOTION:
                // 诊断版 Minima：data0 bit7=1 表示 data1/data2 携带实际执行掩码。
                if (_rxBuffer[3] & 0x80) {
                    _minimaDiag = true;
                    _minimaMask = static_cast<uint16_t>(_rxBuffer[4]) |
                                  (static_cast<uint16_t>(_rxBuffer[5]) << 8);
                    _minimaMaskRxMs = millis();
                    _maskSamplePending = true;
                }
                processMotionStatus(_rxBuffer[3] & 0x07);
                break;
            case STATUS_PARAM: {
                const uint8_t id = _rxBuffer[3];
                const uint16_t val = static_cast<uint16_t>(_rxBuffer[4]) |
                                     (static_cast<uint16_t>(_rxBuffer[5]) << 8);
                if (id < 16) {
                    _params[id] = val;
                    _paramsSeen = true;
                }
                // 打成固定格式，PC 控制台据此显示参数表。
                g_dbg->printf("[PARAM] %u %s = %u\r\n", static_cast<unsigned>(id),
                              id < P_COUNT ? PARAM_NAMES[id] : "?", static_cast<unsigned>(val));
                break;
            }
            case STATUS_HEARTBEAT:
                if (_minimaDiag) {
                    _minimaBadFrames = static_cast<uint16_t>(_rxBuffer[3]) |
                                       (static_cast<uint16_t>(_rxBuffer[4]) << 8);
                    _minimaLoopMaxMs = _rxBuffer[5];
                }
                processHeartbeat();
                break;
            default:
                break;
        }
    }
}

uint8_t StatusDisplay::getLastMotionStatus() const {
    return _lastMotionStatus;
}

bool StatusDisplay::hasRecentHeartbeat(uint32_t nowMs, uint32_t timeoutMs) const {
    return _lastHeartbeat != 0 && nowMs - _lastHeartbeat <= timeoutMs;
}

void StatusDisplay::processMotionStatus(uint8_t status) {
    if (status == _lastMotionStatus) {
        return;
    }
    _lastMotionStatus = status;

    g_dbg->print("<- Minima: ");
    if (status & 0x01) g_dbg->print("FWD ");
    if (status & 0x02) g_dbg->print("TURN ");
    if (status & 0x04) g_dbg->print("BUOY ");
    if (status == 0) g_dbg->print("IDLE");
    g_dbg->println();
}

void StatusDisplay::processHeartbeat() {
    _lastHeartbeat = millis();
}
