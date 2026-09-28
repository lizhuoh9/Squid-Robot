/**********************************************************************
 * MotionParams.h
 *
 * 运动时序参数表：全部存在执行器本地（EEPROM 持久化），断电保留。
 *
 * ESP32 只下发"意图"(前进开始/停止、左转、浮沉方向+力度)，具体节拍由执行器
 * 本地按这些参数生成 —— 无线/串口抖动或丢帧都不会打乱阀门时序。
 * 改参数：ESP32 发 CMD_SET_PARAM（立即生效），再发 CMD_SAVE_PARAMS 落盘。
 *********************************************************************/

#ifndef MINIMA_MOTION_PARAMS_H
#define MINIMA_MOTION_PARAMS_H

#include <Arduino.h>

// 参数编号（与 ESP32 的 Board.h 必须一致）。
enum ParamId : uint8_t {
    P_FWD_INTERVAL   = 0,   // 前进：阀 A/B 同时开关的切换周期 ms
    P_FWD_BAL_DELAY  = 1,   // 前进：停止后多久开始平衡 ms
    P_FWD_BAL_TIME   = 2,   // 前进：普通停止的平衡总时长 ms
    P_FWD_BAL_ALT    = 3,   // 前进：平衡时每路阀打开时长 ms
    P_TURN_DURATION  = 4,   // 转向：推进持续时间 ms
    P_TURN_BAL_DELAY = 5,   // 转向：结束后多久开始平衡 ms
    P_TURN_BAL_TIME  = 6,   // 转向：平衡总时长 ms
    P_TURN_BAL_ALT   = 7,   // 转向：平衡时每路阀打开时长 ms
    P_GLOBAL_BAL_TIME= 8,   // 急停全局平衡总时长 ms
    P_GLOBAL_BAL_ALT = 9,   // 急停全局平衡每路阀打开时长 ms
    P_BUOY_BAL_ALT   = 10,  // 浮沉平衡：E/F 单阀交替周期 ms
    P_VALVE_GUARD    = 11,  // 浮沉阀门换向后关泵的保护间隔 ms
    P_PWM_PERIOD_100US = 12,// 浮力泵软件 PWM 周期，单位 100us（200 = 20ms）
    P_LINK_TIMEOUT   = 13,  // 【已废弃】原链路失效保护超时；2026-09-21 删除该保护，
                            // 槽位保留仅为不打乱 EEPROM 里已存参数的编号。
    P_COUNT          = 14
};

struct MotionParams {
    uint16_t v[P_COUNT];

    uint16_t operator[](uint8_t id) const { return id < P_COUNT ? v[id] : 0; }

    void loadDefaults();
    // 从 EEPROM 读；magic/版本/校验不对就用默认值并返回 false。
    bool load();
    void save() const;
    bool set(uint8_t id, uint16_t value);
    void print() const;   // 打到 USB 串口，人工核对用
};

extern MotionParams g_params;

#endif  // MINIMA_MOTION_PARAMS_H
