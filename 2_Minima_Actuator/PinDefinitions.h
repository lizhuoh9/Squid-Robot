/**********************************************************************
 * PinDefinitions.h
 *
 * Minima 侧执行器引脚映射。
 * 命名规则：气泵用 PUMP_x，电磁阀用 VALVE_x（字母与硬件标签一致）。
 *********************************************************************/

#ifndef MINIMA_PIN_DEFINITIONS_H
#define MINIMA_PIN_DEFINITIONS_H

#define PUMP_A_PIN      2   // 前进推进气泵
#define PUMP_D_PIN      3   // 转向推进气泵
#define PUMP_G_PIN      4   // 浮力控制气泵

#define VALVE_A_PIN     5   // 推进系统电磁阀 a
#define VALVE_B_PIN     6   // 推进系统电磁阀 b

#define VALVE_C_PIN     7   // 转向系统电磁阀 c
#define VALVE_D_PIN     8   // 转向系统电磁阀 d

#define VALVE_E_PIN     9   // 浮力系统电磁阀 e
#define VALVE_F_PIN     10  // 浮力系统电磁阀 f

#endif // MINIMA_PIN_DEFINITIONS_H
