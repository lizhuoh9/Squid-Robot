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
constexpr float kInputMean[kInputDim] = {-4.21091461f, 0.21326302f, -0.0903287604f, -23.0914116f, 215.761642f, -4.32383251f, 0.175572619f, -0.101684928f, -22.4521027f, 215.435623f, -4.42743587f, 0.153473973f, -0.0949205458f, -22.1495209f, 215.235611f, -4.52595901f, 0.145345226f, -0.0787342414f, -22.5211411f, 214.350677f, -4.6254878f, 0.150780827f, -0.058254797f, -23.8362103f, 214.350677f};
constexpr float kInputStd[kInputDim] = {5.66585255f, 2.46876597f, 1.75539494f, 73.2849808f, 35.2179222f, 5.45169926f, 2.46770191f, 1.75653625f, 73.2974396f, 35.6615639f, 5.25388622f, 2.46458578f, 1.75647855f, 73.2872086f, 35.9395142f, 5.06814051f, 2.46135235f, 1.75185752f, 72.9449921f, 38.0401764f, 4.88651323f, 2.45178366f, 1.74284685f, 72.5259171f, 38.0352783f};
}  // namespace depth_model_config
