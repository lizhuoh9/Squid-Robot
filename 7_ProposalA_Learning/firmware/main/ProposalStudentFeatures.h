#pragma once
#include <cstdint>
#include <cmath>
#include "ProposalStudentConfig.h"

// Portable feature builder shared by the worker request and host tests.
// Histories are oldest first and cover completed, exact 100ms intervals.
class ProposalStudentFeatures {
public:
    void reset() {
        started = false;
        currentForward = previousForward = false;
        count = 0;
        previousMs = windowStart = 0;
        commandIntegral = forwardIntegral = 0;
        for (int i = 0; i < proposal_model_config::kHistory; ++i) {
            commands[i] = forwards[i] = 0;
            depths[i] = speeds[i] = 0;
        }
        previousDepth = previousSpeed = slotDepth = slotSpeed = 0;
    }

    // direction/PWM describe the command held over the preceding motion tick.
    // forwardActive is the observation at nowMs, so its PREVIOUS value is
    // integrated over that tick. Return true when a timing gap invalidates it.
    bool observe(uint8_t direction, uint8_t pwm, bool forwardActive, uint32_t nowMs,
                 float depth = 0, float speed = 0, bool depthValid = true) {
        if (!depthValid || !std::isfinite(depth) || !std::isfinite(speed)) { reset(); return true; }
        currentForward = forwardActive;
        if (!started) {
            start(forwardActive, nowMs, depth, speed);
            return false;
        }
        const uint32_t elapsed = nowMs - previousMs;
        if (elapsed > 50U) {
            reset();
            start(forwardActive, nowMs, depth, speed);
            return true;
        }
        const float command = pwm / 255.0f * (direction == 1 ? 1.0f : direction == 2 ? -1.0f : 0.0f);
        uint32_t remaining = elapsed;
        while (remaining) {
            const uint32_t room = proposal_model_config::kIntervalMs - (previousMs - windowStart);
            const uint32_t duration = remaining < room ? remaining : room;
            commandIntegral += command * duration;
            forwardIntegral += (previousForward ? 1.0f : 0.0f) * duration;
            previousMs += duration;
            remaining -= duration;
            if (previousMs - windowStart == proposal_model_config::kIntervalMs) {
                for (int i = 0; i < proposal_model_config::kHistory - 1; ++i) {
                    commands[i] = commands[i + 1];
                    forwards[i] = forwards[i + 1];
                    depths[i] = depths[i + 1]; speeds[i] = speeds[i + 1];
                }
                commands[proposal_model_config::kHistory - 1] = commandIntegral / proposal_model_config::kIntervalMs;
                forwards[proposal_model_config::kHistory - 1] = forwardIntegral / proposal_model_config::kIntervalMs;
                // Snapshot at the START of the completed interval, matching
                // training's past-grid causal hold (never the new endpoint).
                depths[proposal_model_config::kHistory - 1] = slotDepth;
                speeds[proposal_model_config::kHistory - 1] = slotSpeed;
                commandIntegral = forwardIntegral = 0;
                windowStart = previousMs;
                slotDepth = previousMs == nowMs ? depth : previousDepth;
                slotSpeed = previousMs == nowMs ? speed : previousSpeed;
                if (count < proposal_model_config::kHistory) ++count;
            }
        }
        previousForward = forwardActive;
        previousDepth = depth; previousSpeed = speed;
        if (nowMs == windowStart) { slotDepth = depth; slotSpeed = speed; }
        return false;
    }

    bool ready() const { return count == proposal_model_config::kHistory; }

    void fill(float* values, float error, float speed, float accel, float bias, float depth = 0) const {
        values[0] = error; values[1] = speed; values[2] = accel; values[3] = bias;
        for (int i = 0; i < proposal_model_config::kHistory; ++i) {
            values[proposal_model_config::kCommandOffset + i] = commands[i];
            values[proposal_model_config::kForwardHistoryOffset + i] = forwards[i];
            if (proposal_model_config::kInputDim == 125) {
                values[65 + 2*i] = depths[i] - depth;
                values[65 + 2*i + 1] = speeds[i];
            }
        }
        values[proposal_model_config::kForwardIndex] = currentForward ? 1.0f : 0.0f;
    }

private:
    void start(bool forwardActive, uint32_t nowMs, float depth, float speed) {
        started = true;
        currentForward = previousForward = forwardActive;
        previousMs = windowStart = nowMs;
        previousDepth = slotDepth = depth; previousSpeed = slotSpeed = speed;
    }
    float commands[proposal_model_config::kHistory] = {};
    float forwards[proposal_model_config::kHistory] = {};
    float depths[proposal_model_config::kHistory] = {}, speeds[proposal_model_config::kHistory] = {};
    float previousDepth = 0, previousSpeed = 0, slotDepth = 0, slotSpeed = 0;
    float commandIntegral = 0, forwardIntegral = 0;
    uint32_t previousMs = 0, windowStart = 0;
    int count = 0;
    bool started = false, currentForward = false, previousForward = false;
};
