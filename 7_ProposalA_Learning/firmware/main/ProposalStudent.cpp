#include "ProposalStudent.h"
#include "ResidualRLConfig.h"
#include "ResidualRLFeatures.h"
#include "dl_tensor_base.hpp"
#include <cmath>
#include <new>
#include <atomic>
#include <cstring>
#include "freertos/task.h"

extern const uint8_t proposal_student_blob[] asm("_binary_proposal_student_espdl_start");
extern const uint8_t residual_rl_blob[] asm("_binary_residual_rl_espdl_start");
namespace cfg = proposal_model_config;
namespace rlcfg = residual_rl_config;
namespace { std::atomic<bool> runtimeReady{false}, rlRuntimeReady{false}; }
bool proposalStudentRuntimeReady() { return runtimeReady.load(); }
bool residualRLRuntimeReady() { return rlRuntimeReady.load(); }
static_assert(cfg::kInputDim == 125 && rlcfg::kInputDim == 128, "RL input schema mismatch");
static_assert((cfg::kInputDim == 65 || cfg::kInputDim == 125) && cfg::kHistory == 30 && cfg::kIntervalMs == 100 &&
              cfg::kCommandOffset == 4 && cfg::kForwardIndex == 4 + cfg::kHistory &&
              cfg::kForwardHistoryOffset == cfg::kForwardIndex + 1 &&
              cfg::kForwardHistoryOffset + cfg::kHistory == 65,
              "Student feature and history contract mismatch");

bool ProposalStudent::begin() {
    // Retain the paired ESP-DL blob in flash even when deployment is disabled.
    const volatile uint8_t* embedded = proposal_student_blob;
    (void)*embedded;
    // kEnabled is an explicit experimental override, not a successful gate.
    // The exporter defaults it back to false when generating a new model.
    if (!cfg::kEnabled) return false;
    model = new (std::nothrow) dl::Model(reinterpret_cast<const char*>(proposal_student_blob),
                                     fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    if (!model) return false;
    const auto inputs = model->get_inputs(), outputs = model->get_outputs();
    if (inputs.size() != 1 || outputs.size() != 1) return false;
    input = inputs.begin()->second; output = outputs.begin()->second;
    if (!input || !output || input->dtype != dl::DATA_TYPE_INT8 ||
        output->dtype != dl::DATA_TYPE_INT8 || input->get_size() != cfg::kInputDim ||
        output->get_size() != 1 || DL_SCALE(input->exponent) != cfg::kInputScale ||
        DL_SCALE(output->exponent) != cfg::kOutputScale) return false;
    if (rlcfg::kEnabled && std::strcmp(rlcfg::kBaseCheckpointSha256, cfg::kCheckpointSha256) == 0) {
        rlModel = new (std::nothrow) dl::Model(reinterpret_cast<const char*>(residual_rl_blob), fbs::MODEL_LOCATION_IN_FLASH_RODATA);
        if (rlModel) {
            const auto in = rlModel->get_inputs(), out = rlModel->get_outputs();
            if (in.size() == 1 && out.size() == 1) {
                rlInput = in.begin()->second; rlTensor = out.begin()->second;
                rlReady = rlInput && rlTensor && rlInput->dtype == dl::DATA_TYPE_INT8 &&
                    rlTensor->dtype == dl::DATA_TYPE_INT8 && rlInput->get_size() == 128 && rlTensor->get_size() == 1 &&
                    DL_SCALE(rlInput->exponent) == rlcfg::kInputScale && DL_SCALE(rlTensor->exponent) == rlcfg::kLogitScale;
            }
            if (!rlReady) { delete rlModel; rlModel = nullptr; rlInput = rlTensor = nullptr; }
        }
    }
    requests = xQueueCreate(1, sizeof(Request)); results = xQueueCreate(1, sizeof(Result));
    if (!requests || !results) return false;
    ready = xTaskCreatePinnedToCore(task, "proposal_nn", 8192, this, 2, nullptr, 0) == pdPASS;
    runtimeReady.store(ready);
    rlRuntimeReady.store(ready && rlReady);
    return ready;
}

void ProposalStudent::reset() {
    features.reset();
    invalidateResults();
}

void ProposalStudent::invalidateResults() {
    ++generation;
    resultMs = 0;
    correction = 0;
    rlCorrection = 0; rlResultValid = rlApplied = false;
    resultValid = false; applied = false;
    if (results) xQueueReset(results);
    // A running worker may finish an old request; generation rejects that result.
}

void ProposalStudent::observe(uint8_t direction, uint8_t pwm, bool forwardActive, uint32_t nowMs,
                              bool depthValid, float depth, float speed) {
    if (!ready || !cfg::kEnabled) return;
    if (features.observe(direction, pwm, forwardActive, nowMs, depth, speed, depthValid)) invalidateResults();
}

float ProposalStudent::predict(float error, float speed, float accel, float bias, uint32_t nowMs, float depth) {
    applied = false;
    rlApplied = false;
    if (!ready || !cfg::kEnabled) return 0;
    if (!std::isfinite(error) || !std::isfinite(speed) || !std::isfinite(accel) || !std::isfinite(bias) ||
        std::fabs(error) > cfg::kGuardError || std::fabs(speed) > cfg::kGuardSpeed) {
        reset(); return 0;
    }
    if (!features.ready()) return 0;
    Request request{};
    features.fill(request.features, error, speed, accel, bias, depth);
    for (int i = 0; i < cfg::kInputDim; ++i) {
        const float value = (request.features[i]-cfg::kInputMean[i])/cfg::kInputStd[i];
        if (!std::isfinite(value) || std::fabs(value) > cfg::kGuardNormalized) { reset(); return 0; }
        request.features[i] = value;
    }
    Result result{};
    if (xQueueReceive(results, &result, 0) == pdTRUE && result.generation == generation) {
        correction = result.correction; resultMs = result.stamp; resultValid = result.valid;
        rlCorrection = result.rlCorrection; rlResultValid = result.rlValid;
    }
    const bool fresh = resultMs && nowMs-resultMs <= 250U;
    applied = fresh && resultValid;
    rlApplied = fresh && rlResultValid;
    request.depth = depth; request.previousRL = rlApplied ? rlCorrection : 0;
    request.stamp = nowMs; request.generation = generation;
    xQueueOverwrite(requests, &request); // Nonblocking; newest observation wins.
    return applied ? correction : 0;
}

void ProposalStudent::task(void* self) { static_cast<ProposalStudent*>(self)->run(); }
void ProposalStudent::run() {
    Request request{};
    while (true) {
        if (xQueueReceive(requests, &request, portMAX_DELAY) != pdTRUE) continue;
        auto* values = static_cast<int8_t*>(input->data);
        bool valid = true;
        const float rescale = DL_RESCALE(input->exponent);
        for (int i = 0; i < cfg::kInputDim; ++i) {
            if (std::fabs(request.features[i]*rescale) > 127.0f) { valid = false; break; }
            values[i] = dl::quantize<int8_t>(request.features[i], rescale);
        }
        float value = 0;
        if (valid) {
            model->run();
            value = dl::dequantize<int8_t>(static_cast<const int8_t*>(output->data)[0], DL_SCALE(output->exponent));
            if (!std::isfinite(value)) { value = 0; valid = false; }
        }
        value = value < -cfg::kLimit ? -cfg::kLimit : value > cfg::kLimit ? cfg::kLimit : value;
        float rlValue = 0; bool rlValid = false;
        if (valid && rlReady) {
            float observation[128];
            const float error = request.features[0] * cfg::kInputStd[0] + cfg::kInputMean[0];
            const float usedNN = (error <= 0 && value < 0) ? 0 : value;
            rlValid = residual_rl::fillObservation(request.features, usedNN, request.previousRL, request.depth, observation);
            auto* q = static_cast<int8_t*>(rlInput->data);
            const float scale = DL_RESCALE(rlInput->exponent);
            for (int i = 0; rlValid && i < 128; ++i) {
                if (std::fabs(observation[i] * scale) > 127) { rlValid = false; break; }
                q[i] = dl::quantize<int8_t>(observation[i], scale);
            }
            if (rlValid) {
                rlModel->run();
                const float logit = dl::dequantize<int8_t>(static_cast<const int8_t*>(rlTensor->data)[0], DL_SCALE(rlTensor->exponent));
                rlValid = std::isfinite(logit);
                if (rlValid) rlValue = residual_rl::bound(std::tanh(logit) * rlcfg::kLimit, rlcfg::kLimit);
            }
        }
        const Result result{value, rlValue, request.stamp, request.generation, valid, rlValid};
        xQueueOverwrite(results, &result);
    }
}
