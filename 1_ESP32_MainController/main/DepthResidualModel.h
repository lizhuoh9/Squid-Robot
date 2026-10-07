/**********************************************************************
 * DepthResidualModel.h
 *
 * ESP-DL inference for the depth-controller residual model embedded in flash.
 * Without a valid embedded model, the controller uses zero correction.
 **********************************************************************/

#ifndef SQUID_DEPTH_RESIDUAL_MODEL_H
#define SQUID_DEPTH_RESIDUAL_MODEL_H

#include <cstdint>

#include "DepthResidualModelConfig.h"
#include "dl_model_base.hpp"

class DepthResidualModel {
public:
    DepthResidualModel();

    bool begin();
    void reset();
    bool ready() const;

    float predict(float depthErrorCm,
                  float depthSpeedCmS,
                  float depthAccelCmS2,
                  float baseOutput,
                  float buoyancyPwm,
                  uint32_t nowMs);

private:
    float _history[depth_model_config::kWindow][depth_model_config::kFeaturesPerFrame];
    uint8_t _count;
    uint32_t _lastFrameMs;
    float _lastPrediction;
    dl::Model* _model;
    dl::TensorBase* _input;
    dl::TensorBase* _output;
    bool _ready;
};

#endif  // SQUID_DEPTH_RESIDUAL_MODEL_H
