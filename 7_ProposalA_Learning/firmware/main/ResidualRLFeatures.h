#pragma once
#include <cmath>

namespace residual_rl {
inline float bound(float x, float limit) {
    return x < -limit ? -limit : x > limit ? limit : x;
}
// Same order as residual_rl.Env.observation. Base features are already
// normalized and guard-checked; history is causal and shared with the NN.
inline bool fillObservation(const float* normalized, float nn, float previous,
                            float depth, float* result) {
    if (!std::isfinite(nn) || !std::isfinite(previous) || !std::isfinite(depth) || depth < 0 || depth > 100) return false;
    for (int i = 0; i < 125; ++i) {
        if (!std::isfinite(normalized[i]) || std::fabs(normalized[i]) > 6) return false;
        result[i] = normalized[i];
    }
    result[125] = bound(nn, 0.8f) / 0.8f;
    result[126] = bound(previous, 0.8f) / 0.8f;
    result[127] = depth / 40.0f;
    return true;
}
inline float combine(float nn, float rl) {
    return bound(nn, 0.8f) + bound(rl, 0.8f); // deliberately no sum clipping
}
} // namespace residual_rl
