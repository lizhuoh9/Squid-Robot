/**********************************************************************
 * DepthController.h
 *
 * Depth-hold controller, "Proposal A":
 *   desired-acceleration PD  +  direction-specific actuator inversion
 *   +  depth-dependent heaviness feed-forward with a slow integral
 *   +  time-proportional modulation below the pump's minimum duty.
 * Manual, balance and safety behaviour is unchanged from the previous
 * controller, and so is the public interface.
 *
 * Sign convention: depth, velocity and acceleration are positive DOWNWARD.
 *********************************************************************/

#ifndef SQUID_DEPTH_CONTROLLER_H
#define SQUID_DEPTH_CONTROLLER_H

#include <cstdint>

#include <Board.h>

// Tuning constants. Values come from the 2026-10-06/07 tank logs; replace the
// plant numbers (G_SINK, G_RISE, A_REST_REF, ALPHA) with the open-loop test results.
namespace depth_cfg {
// Closed-loop shape: a_des = -KP*e - KD*v  (wn = 0.30 rad/s, zeta = 1.0)
constexpr float KP = 0.09f;             // [cm/s^2 per cm]     = wn^2
constexpr float KD = 0.60f;             // [cm/s^2 per cm/s]   = 2*zeta*wn
// Slow correction of the heaviness estimate. Not reset when the target changes.
constexpr float KI = 0.01f;             // [cm/s^2 per cm*s]
constexpr float I_LIMIT = 1.5f;         // max |KI * integral|  [cm/s^2]
// Steady acceleration produced by a full-duty pump, per direction.
constexpr float G_SINK = 0.75f;         // [cm/s^2]  (tauW * Ks = 1.5 * 0.5)
constexpr float G_RISE = 4.5f;          // [cm/s^2]  (tauW * Kr = 1.5 * 3.0, deliberately on the high side)
// Heaviness when the pump is idle: sinking acceleration at DEPTH_REF, growing with depth.
constexpr float A_REST_REF = 1.1f;      // [cm/s^2]
constexpr float DEPTH_REF = 40.0f;      // [cm]
constexpr float ALPHA = 0.03f;          // [cm/s^2 per cm]
// Actuator handling.
constexpr float DUTY_MIN = 80.0f / 255.0f;  // lowest duty the pump runs at (PWM 80)
constexpr uint32_t MOD_WINDOW_MS = 1000;    // modulation window below DUTY_MIN
constexpr float FLIP_MIN_DUTY = 0.05f;      // smaller opposite requests do not switch direction
}  // namespace depth_cfg

class DepthController {
public:
    DepthController();

    void begin();
    // Call every motion tick (5 ms). The control law advances only when depthFresh
    // (a new MS5837 sample, ~12 Hz); the low-duty modulation runs every tick.
    void update(bool depthValid,
                bool depthFresh,
                float filteredDepthCm,
                float depthSpeedCmS,
                float depthAccelCmS2,
                uint32_t nowMs);

    void setTargetDepth(float targetDepthCm);
    void holdCurrentDepth();
    void clearTarget();
    bool isHoldingTarget() const;
    bool isBalancing() const { return _balancing; }

    void manualAscend();
    void manualDescend();
    void manualStop();
    void forceBalance();

    void resetAfterCalibration();

    float getFilteredDepthCm() const;
    float getSpeedCmS() const;
    float getAccelerationCmS2() const;
    float getTargetDepthCm() const;
    float getControlOutput() const;   // signed duty x100 (+sink / -rise), before modulation
    float getPidRawOutput() const;    // desired acceleration a_des [cm/s^2]
    float getPidBaseOutput() const;   // heaviness estimate (feed-forward + integral) [cm/s^2]
    float getResidualOutput() const;  // always 0: the neural residual is not used
    float getTotalOutput() const;     // same as getControlOutput()
    bool isOutputSaturated() const;

    uint8_t getBuoyancyDirection() const;   // intent: stop / rise / sink / balance
    uint8_t getBuoyancyPwm() const;         // pump PWM 0..255
    uint8_t getManualDirection() const;

private:
    void computeDuty(uint32_t nowMs);
    void applyModulatedOutput(uint32_t nowMs);
    void stopBuoyancyOutput();

    bool _holdingTarget;

    float _targetDepthCm;
    float _depthFilt;
    float _speedCmS;
    float _accelCmS2;       // telemetry only; the Kalman acceleration lags ~0.9 s
    float _integ;           // integral of depth error [cm*s]
    float _dutyCmd;         // signed duty [-1, 1], +sink / -rise
    int8_t _lastDir;        // direction of the last non-zero request: +1 sink, -1 rise
    float _aDes;
    float _bias;
    float _controlOutput;
    bool  _outputSaturated;

    uint8_t  _manualDirection;
    uint8_t  _buoyancyDirection;
    uint8_t  _buoyancyPwm;
    uint32_t _lastControlUpdateMs;
    uint32_t _modWindowStartMs;
    uint32_t _modOnUntilMs;
    bool     _balancing;
    uint32_t _balanceStartMs;
    uint32_t _balanceEndMs;
};

#endif  // SQUID_DEPTH_CONTROLLER_H
