// Host-build stub: only the constants DepthController / Calibration need (values copied from main/Board.h).
#ifndef SQUID_BOARD_STUB_H
#define SQUID_BOARD_STUB_H
#include <cstddef>
#include <cstdint>
constexpr uint8_t SENSOR_FRONT = 0;
constexpr uint8_t SENSOR_LEFT  = 1;
constexpr uint8_t SENSOR_RIGHT = 2;
constexpr uint8_t BUOYANCY_STOP    = 0;
constexpr uint8_t BUOYANCY_ASCEND  = 1;
constexpr uint8_t BUOYANCY_DESCEND = 2;
constexpr uint8_t BUOYANCY_BALANCE = 3;
#endif
