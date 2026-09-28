/**********************************************************************
 * MotionControl.cpp
 *
 * Minima 底层执行器驱动。
 *
 * applyMask()      — 接收 ESP32 下发的执行器掩码，立即写引脚。
 * applyBuoyancy()  — 接收浮沉方向和 PWM，存入请求，由 update() 执行。
 * update()         — 每帧调用，仅处理浮沉软件 PWM 输出（气泵 + 阀门）。
 *                    前进/转向引脚由 applyMask() 负责，无需重复写。
 *********************************************************************/

#include "MotionControl.h"

#include "MotionParams.h"

namespace {
// 浮力泵软件 PWM 周期改由参数表给（P_PWM_PERIOD_100US，单位 100us）。
}

MotionController::MotionController()
    : _mask(0),
      _buoyancyValves(0),
      _buoyancyPwm(0) {}

void MotionController::begin() {
    pinMode(PUMP_A_PIN,  OUTPUT);
    pinMode(VALVE_A_PIN, OUTPUT);
    pinMode(VALVE_B_PIN, OUTPUT);
    pinMode(PUMP_D_PIN,  OUTPUT);
    pinMode(VALVE_C_PIN, OUTPUT);
    pinMode(VALVE_D_PIN, OUTPUT);
    pinMode(PUMP_G_PIN,  OUTPUT);
    pinMode(VALVE_E_PIN, OUTPUT);
    pinMode(VALVE_F_PIN, OUTPUT);

    emergencyStopAll();
}

void MotionController::applyMask(uint16_t mask) {
    _mask = mask;
    writeOutputs();
}

void MotionController::applyBuoyancy(uint8_t valveBits, uint8_t pwm) {
    _buoyancyValves = valveBits & 0x03;
    _buoyancyPwm    = pwm;
}

void MotionController::update() {
    // 前进/转向引脚由 applyMask() 在收到帧时立即写入，此处只处理浮沉 PWM。
    writeBuoyancyOutputs(millis());
}

void MotionController::emergencyStopAll() {
    _mask = 0;
    _buoyancyValves = 0;
    _buoyancyPwm = 0;
    writeOutputs();
}

uint16_t MotionController::getMask() const {
    return _mask;
}

bool MotionController::isForwardActive() const {
    return (_mask & ACT_FORWARD_GROUP) != 0;
}

bool MotionController::isTurnActive() const {
    return (_mask & ACT_TURN_GROUP) != 0;
}

bool MotionController::isBuoyancyActive() const {
    return _buoyancyPwm > 0 || _buoyancyValves != 0;
}

bool MotionController::isAnyActive() const {
    return _mask != 0 || isBuoyancyActive();
}

uint8_t MotionController::getBuoyancyValves() const {
    return _buoyancyValves;
}

uint8_t MotionController::getBuoyancyPwm() const {
    return _buoyancyPwm;
}

void MotionController::printStatus() {
    Serial.print(F("Mask=0x"));
    Serial.print(_mask, HEX);
    Serial.print(F(" | "));

    if (isForwardActive()) {
        Serial.print(F("FWD "));
    }

    if (isTurnActive()) {
        Serial.print(F("TURN "));
    }

    if (isBuoyancyActive()) {
        Serial.print(F("BUOY("));
        Serial.print(F("EF="));
        Serial.print(_buoyancyValves, BIN);
        Serial.print(',');
        Serial.print(_buoyancyPwm);
        Serial.print(F(") "));
    }

    if (!isAnyActive()) {
        Serial.print(F("IDLE"));
    }

    Serial.println();
}

void MotionController::writeOutputs() {
    // 前进子系统：泵 + 阀 a/b
    digitalWrite(PUMP_A_PIN,  (_mask & ACT_FORWARD_PUMP)    != 0 ? HIGH : LOW);
    digitalWrite(VALVE_A_PIN, (_mask & ACT_FORWARD_VALVE_A) != 0 ? HIGH : LOW);
    digitalWrite(VALVE_B_PIN, (_mask & ACT_FORWARD_VALVE_B) != 0 ? HIGH : LOW);

    // 转向子系统：泵 + 阀 c/d
    digitalWrite(PUMP_D_PIN,  (_mask & ACT_TURN_PUMP)       != 0 ? HIGH : LOW);
    digitalWrite(VALVE_C_PIN, (_mask & ACT_TURN_VALVE_C)    != 0 ? HIGH : LOW);
    digitalWrite(VALVE_D_PIN, (_mask & ACT_TURN_VALVE_D)    != 0 ? HIGH : LOW);

    // 浮力子系统由 writeBuoyancyOutputs() 单独处理。
    writeBuoyancyOutputs(millis());
}

void MotionController::writeBuoyancyOutputs(uint32_t nowMs) {
    (void)nowMs;
    // 阀门：位由 ESP32 直接给定（平衡交替、上浮/下沉的阀门组合都在 ESP32 算好）。
    digitalWrite(VALVE_E_PIN, (_buoyancyValves & 0x01) ? HIGH : LOW);
    digitalWrite(VALVE_F_PIN, (_buoyancyValves & 0x02) ? HIGH : LOW);

    // 泵：软件 PWM（波形生成必须在本地做）。
    const uint32_t periodUs = static_cast<uint32_t>(g_params[P_PWM_PERIOD_100US]) * 100UL;
    const uint32_t dutyUs = (static_cast<uint32_t>(_buoyancyPwm) * periodUs) / 255U;
    const bool pumpOn = dutyUs > 0 && periodUs > 0 && (micros() % periodUs) < dutyUs;
    digitalWrite(PUMP_G_PIN, pumpOn ? HIGH : LOW);
}
