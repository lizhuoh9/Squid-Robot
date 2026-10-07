#include "DepthResidualModel.h"

#include <cmath>
#include <new>

#include "dl_tensor_base.hpp"

extern const uint8_t depth_model_espdl[] asm("_binary_depth_model_espdl_espdl_start");

namespace {
namespace config = depth_model_config;
constexpr float kResidualLimit = config::kResidualLimit;
constexpr int kWindow = config::kWindow;
constexpr int kFeaturesPerFrame = config::kFeaturesPerFrame;
constexpr int kInputDim = config::kInputDim;
static_assert(kFeaturesPerFrame == 5, "Five-feature ordering must match the trained model");
static_assert(config::kSampleIntervalMs > 0, "Model sampling interval must be positive");
static_assert(config::kOutputQuantMin * config::kOutputScale <= -kResidualLimit &&
              config::kOutputQuantMax * config::kOutputScale >= kResidualLimit,
              "INT8 output must represent the complete residual range");

inline float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

}  // namespace

DepthResidualModel::DepthResidualModel()
    : _history{}, _count(0), _lastFrameMs(0), _lastPrediction(0.0f),
      _model(nullptr), _input(nullptr), _output(nullptr), _ready(false) {}

bool DepthResidualModel::begin() {
    if (_ready) return true;
    _model = new (std::nothrow) dl::Model(
        reinterpret_cast<const char*>(depth_model_espdl), fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    if (_model == nullptr) return false;

    const auto inputs = _model->get_inputs();
    const auto outputs = _model->get_outputs();
    if (inputs.empty() || outputs.empty()) return false;
    _input = inputs.begin()->second;
    _output = outputs.begin()->second;
    if (_input == nullptr || _output == nullptr ||
        _input->dtype != dl::DATA_TYPE_INT8 || _output->dtype != dl::DATA_TYPE_INT8 ||
        _input->get_size() != kInputDim || _output->get_size() != 1 ||
        DL_SCALE(_output->exponent) != config::kOutputScale) {
        _input = nullptr;
        _output = nullptr;
        return false;
    }
    _ready = true;
    return true;
}

void DepthResidualModel::reset() {
    for (auto& frame : _history) {
        for (float& value : frame) value = 0.0f;
    }
    _count = 0;
    _lastFrameMs = 0;
    _lastPrediction = 0.0f;
}

bool DepthResidualModel::ready() const {
    return _ready;
}

float DepthResidualModel::predict(float depthErrorCm,
                                  float depthSpeedCmS,
                                  float depthAccelCmS2,
                                  float baseOutput,
                                  float buoyancyPwm,
                                  uint32_t nowMs) {
    if (!ready()) return 0.0f;
#ifdef SQUID_DEPTH_MODEL_BASED
    // CSV telemetry PWM is post-control; firmware's supplied PWM is pre-control.
    // Derive this input from the current base so training/live inputs agree.
    const float baseMagnitude = clampf(std::fabs(baseOutput), 0.0f, 100.0f);
    buoyancyPwm = baseMagnitude < 8.0f ? 0.0f
        : std::floor(80.0f + (baseMagnitude - 8.0f) * 175.0f / 92.0f + 0.5f);
    if (!std::isfinite(depthErrorCm) || !std::isfinite(depthSpeedCmS) ||
        std::fabs(depthErrorCm) > config::kGuardErrorCm ||
        std::fabs(depthSpeedCmS) > config::kGuardSpeedCmS) {
        reset();
        return 0.0f;  // Large initial descent or unsupported speed: base PID only.
    }
#endif
    const float frame[5] = {
        depthErrorCm,
        depthSpeedCmS,
        depthAccelCmS2,
        baseOutput,
        buoyancyPwm,
    };
    for (float value : frame) {
        if (!std::isfinite(value)) {
            reset();
            return 0.0f;
        }
    }


    // The checkpoint was trained on 500 ms telemetry. Sample history on that
    // schedule and hold its correction between samples; PID still uses every
    // fresh depth measurement. Advancing the scheduled time avoids drift.
    if (_count > 0) {
        const uint32_t elapsedMs = nowMs - _lastFrameMs;
        if (elapsedMs < config::kSampleIntervalMs) return _lastPrediction;
        if (elapsedMs >= 2U * config::kSampleIntervalMs) {
            reset();  // A missing history frame must not bridge a sensor gap.
        }
    }
    if (_count == 0) {
        _lastFrameMs = nowMs;
    } else {
        _lastFrameMs += config::kSampleIntervalMs;
    }

    for (int row = kWindow - 1; row > 0; --row) {
        for (int col = 0; col < kFeaturesPerFrame; ++col) {
            _history[row][col] = _history[row - 1][col];
        }
    }
    for (int col = 0; col < kFeaturesPerFrame; ++col) {
        _history[0][col] = frame[col];
    }
    if (_count < kWindow) ++_count;

    if (_count < kWindow) return 0.0f;

    auto* input = static_cast<int8_t*>(_input->data);
    int index = 0;
    for (int row = kWindow - 1; row >= 0; --row) {
        for (int col = 0; col < kFeaturesPerFrame; ++col) {
            const float normalized = (_history[row][col] - config::kInputMean[index]) /
                                     config::kInputStd[index];
#ifdef SQUID_DEPTH_MODEL_BASED
            if (!std::isfinite(normalized) || std::fabs(normalized) > config::kGuardNormalizedAbs) {
                reset();
                return 0.0f;  // Do not extrapolate a learned policy far outside training.
            }
#endif
            input[index] = dl::quantize<int8_t>(
                normalized, DL_RESCALE(_input->exponent));
            ++index;
        }
    }

    _model->run();
    const auto* output = static_cast<const int8_t*>(_output->data);
    const float residual = dl::dequantize<int8_t>(
        output[0], DL_SCALE(_output->exponent));
    _lastPrediction = clampf(residual, -kResidualLimit, kResidualLimit);
    return _lastPrediction;
}
