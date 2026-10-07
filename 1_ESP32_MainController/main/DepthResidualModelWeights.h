/**********************************************************************
 * DepthResidualModelWeights.h
 *
 * Generated model weights are written here by 5_DepthLearning/learning/export.py.
 * The checked-in fallback intentionally contains no learned correction, so a
 * firmware build remains safe before a trained model is installed.
 **********************************************************************/

#ifndef SQUID_DEPTH_RESIDUAL_MODEL_WEIGHTS_H
#define SQUID_DEPTH_RESIDUAL_MODEL_WEIGHTS_H

namespace depth_residual_weights {

constexpr bool kReady = false;
constexpr int kWindow = 5;
constexpr int kFeaturesPerFrame = 5;
constexpr int kInputDim = kWindow * kFeaturesPerFrame;
constexpr int kHidden1 = 24;
constexpr int kHidden2 = 12;

constexpr float kInputMean[kInputDim] = {};
constexpr float kInputStd[kInputDim] = {};
constexpr float kOutputMean = 0.0f;
constexpr float kOutputStd = 1.0f;

constexpr float kW1[kHidden1][kInputDim] = {};
constexpr float kB1[kHidden1] = {};
constexpr float kW2[kHidden2][kHidden1] = {};
constexpr float kB2[kHidden2] = {};
constexpr float kW3[kHidden2] = {};
constexpr float kB3 = 0.0f;

}  // namespace depth_residual_weights

#endif  // SQUID_DEPTH_RESIDUAL_MODEL_WEIGHTS_H
