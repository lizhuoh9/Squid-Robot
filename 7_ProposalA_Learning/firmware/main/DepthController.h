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
#include "ProposalStudent.h"

// Tuning constants. Values come from the 2026-10-06/07 tank logs; replace the
// plant numbers (G_SINK, G_RISE, A_REST_REF, ALPHA) with the open-loop test results.
namespace depth_cfg {
// Original PD outside the target-crossing braking zone.
constexpr float KP = 0.081f;            // [cm/s^2 per cm]
constexpr float KD = 0.54f;             // [cm/s^2 per cm/s]
constexpr float APPROACH_SPEED_MAX = 1.5f; // [cm/s], only in the braking zone
constexpr float APPROACH_LOOKAHEAD_S = 1.0f; // constant-velocity delay allowance
constexpr float BRAKE_SPEED_MIN = 0.3f; // [cm/s], avoid gating on near-zero speed noise
constexpr float BRAKE_ACCEL_EST = 0.6f; // [cm/s^2], initial stopping-distance assumption
constexpr float BRAKE_MARGIN_CM = 0.5f;
constexpr float BRAKE_BLEND_CM = 0.5f; // ramp in at the edge of the braking zone
constexpr float OUTWARD_GUARD_ERROR_CM = 2.0f; // block learned acceleration away from target
constexpr float PEAK_GUARD_START_CM = 1.0f; // start blending before estimated excursion reaches 2 cm
constexpr float PEAK_BRAKE_EXTRA = 0.25f; // [cm/s^2], bounded extra outward-motion braking
// Slow correction of the heaviness estimate. Not reset when the target changes.
constexpr float KI = 0.009f;            // [cm/s^2 per cm*s]
constexpr float I_LIMIT = 1.5f;         // max |KI * integral|  [cm/s^2]
// Steady acceleration produced by a full-duty pump, per direction.
constexpr float G_SINK = 0.75f;         // [cm/s^2]  (tauW * Ks = 1.5 * 0.5)
constexpr float G_RISE = 4.5f;          // [cm/s^2]  (tauW * Kr = 1.5 * 3.0, deliberately on the high side)
// Heaviness when the pump is idle: sinking acceleration at DEPTH_REF, growing with depth.
constexpr float A_REST_REF = 1.1f;      // [cm/s^2]
constexpr float DEPTH_REF = 40.0f;      // [cm]
constexpr float ALPHA = 0.03f;          // [cm/s^2 per cm]
// Actuator handling.
constexpr float HOLD_SINK_DUTY_MAX = 0.35f; // maximum sink duty at/near target or while deep
constexpr float HOLD_SINK_BAND_CM = 2.0f;
constexpr float SINK_RECOVERY_FULL_CM = 5.0f; // full sink duty allowed this far shallow
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
                bool forwardActive,
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
    float getResidualOutput() const;  // bounded effort correction [cm/s^2]; 0 when gate disabled
    bool isResidualApplied() const;   // valid NN result used in the current duty calculation, even if zero
    float getRLResidualOutput() const;
    bool isRLResidualApplied() const;
    float getTotalOutput() const;     // same as getControlOutput()
    bool isOutputSaturated() const;

    uint8_t getBuoyancyDirection() const;   // intent: stop / rise / sink / balance
    uint8_t getBuoyancyPwm() const;         // pump PWM 0..255
    uint8_t getManualDirection() const;

private:
    ProposalStudent _student;
    float _residual = 0.0f;
    bool _residualApplied = false;
    float _rlResidual = 0.0f;
    bool _rlResidualApplied = false;
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
