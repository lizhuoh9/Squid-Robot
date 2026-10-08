/**********************************************************************
 * Calibration.h  —  方向映射（整个固件里唯一需要改方向的地方）
 *
 * 左右超声波、左右转向、浮沉上下，取决于实际接线和气路怎么装，装反了是硬件
 * 问题不是逻辑问题。所以全部集中到这里：哪个反了改这里，别去动
 * AutoNavigator / DemoMode / CommandHandler / DepthController 里的判断逻辑。
 *
 * 校准方法：
 *   超声波   发 g 看读数，用手挡左侧，看是不是 "Left" 那一路在变
 *   转向     发 a 看是不是真的左转
 *   浮沉     发 k 看是不是真的上浮
 *********************************************************************/

#ifndef SQUID2_CALIBRATION_H
#define SQUID2_CALIBRATION_H

#include <cstdint>

#include "Board.h"

namespace cal {

// ── 左右超声波 ────────────────────────────────────────────────────
// 2026-09-21 实测：接反了，左右互换。
constexpr bool FLIP_ULTRASONIC = true;

// ── 左右转向 ──────────────────────────────────────────────────────
// 2026-09-21 实测：正常。
constexpr bool FLIP_TURN = false;

// ── 浮沉 ──────────────────────────────────────────────────────────
// 2026-09-21 实测（用户确认）：协议常量的名字和物理效果是反的。
//   BUOYANCY_ASCEND （阀 E/F 全关 + 泵）= 吸气 → 实际【下沉】
//   BUOYANCY_DESCEND（阀 E/F 全开 + 泵）= 排气 → 实际【上浮】
// 改常量名要同时动执行器固件、收益不大，所以在这里一次性纠正：
// 全固件只认下面这两个名字，不要再直接写 BUOYANCY_ASCEND/DESCEND。
constexpr uint8_t BUOY_SINK = BUOYANCY_ASCEND;    // 下沉（吸气）
constexpr uint8_t BUOY_RISE = BUOYANCY_DESCEND;   // 上浮（排气）
// ─────────────────────────────────────────────────────────────────

// 逻辑左/右 → 实际传感器编号。前视不受影响。
constexpr uint8_t sensorLeft()  { return FLIP_ULTRASONIC ? SENSOR_RIGHT : SENSOR_LEFT; }
constexpr uint8_t sensorRight() { return FLIP_ULTRASONIC ? SENSOR_LEFT  : SENSOR_RIGHT; }

// 逻辑"左转"是否应该发给左转子系统。
constexpr bool turnLeftIsLeft() { return !FLIP_TURN; }

}  // namespace cal

#endif  // SQUID2_CALIBRATION_H
