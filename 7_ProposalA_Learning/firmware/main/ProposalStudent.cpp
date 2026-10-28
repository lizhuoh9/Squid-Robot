#include "ProposalStudent.h"
#include "dl_tensor_base.hpp"
#include <cmath>
#include <new>
#include <atomic>
#include "freertos/task.h"

extern const uint8_t proposal_student_blob[] asm("_binary_proposal_student_espdl_start");
namespace cfg = proposal_model_config;
namespace { std::atomic<bool> runtimeReady{false}; }
bool proposalStudentRuntimeReady() { return runtimeReady.load(); }
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
    requests = xQueueCreate(1, sizeof(Request)); results = xQueueCreate(1, sizeof(Result));
    if (!requests || !results) return false;
    ready = xTaskCreatePinnedToCore(task, "proposal_nn", 6144, this, 2, nullptr, 0) == pdPASS;
    runtimeReady.store(ready);
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
    request.stamp = nowMs; request.generation = generation;
    xQueueOverwrite(requests, &request); // Nonblocking; newest observation wins.
    Result result{};
    if (xQueueReceive(results, &result, 0) == pdTRUE && result.generation == generation) {
        correction = result.correction; resultMs = result.stamp; resultValid = result.valid;
    }
    if (!resultMs || nowMs-resultMs > 250U) return 0;
    applied = resultValid;
    return correction;
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
        const Result result{value, request.stamp, request.generation, valid};
        xQueueOverwrite(results, &result);
    }
}
