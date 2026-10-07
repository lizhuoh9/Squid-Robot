// Generated with depth_model_espdl.espdl. Keep this header and model paired.
#pragma once
#include <cstdint>

namespace depth_model_config {
constexpr int kWindow = 5;
constexpr int kFeaturesPerFrame = 5;
constexpr int kInputDim = kWindow * kFeaturesPerFrame;
constexpr uint32_t kSampleIntervalMs = 500;
constexpr float kResidualLimit = 20.0f;
constexpr float kOutputScale = 0.25f;
constexpr int kOutputQuantMin = -128;
constexpr int kOutputQuantMax = 127;
constexpr float kInputMean[kInputDim] = {6.44197607f, 0.255975425f, -0.0581715517f, 20.0879383f, 210.816177f, 6.30905199f, 0.231531814f, -0.0694558471f, 20.0879383f, 210.816177f, 6.18039846f, 0.220311284f, -0.0641421378f, 19.3036251f, 210.816177f, 6.04966164f, 0.225465626f, -0.0471446104f, 18.5920296f, 210.678925f, 5.91892433f, 0.229200974f, -0.0388750061f, 17.8077164f, 210.678925f};
constexpr float kInputStd[kInputDim] = {9.33473873f, 1.86407077f, 1.43611681f, 75.7808533f, 31.5764675f, 9.28807354f, 1.87683392f, 1.45155478f, 75.7808533f, 31.5764675f, 9.23647881f, 1.89361227f, 1.45537031f, 75.9843445f, 31.5764675f, 9.18913651f, 1.88979471f, 1.45270598f, 76.0994492f, 31.6708679f, 9.14634228f, 1.87549889f, 1.4572202f, 76.2867126f, 31.6708698f};
}  // namespace depth_model_config

// This model uses causal base PWM, not telemetry PWM.
#define SQUID_DEPTH_MODEL_BASED 1
namespace depth_model_config {
constexpr float kGuardErrorCm = 15.0f;
constexpr float kGuardSpeedCmS = 5.0f;
constexpr float kGuardNormalizedAbs = 6.0f;
}
