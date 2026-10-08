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
static_assert(cfg::kInputDim == 34 && cfg::kHistory == 30 && cfg::kIntervalMs == 100,
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
        output->dtype != dl::DATA_TYPE_INT8 || input->get_size() != 34 ||
        output->get_size() != 1 || DL_SCALE(output->exponent) != cfg::kOutputScale) return false;
    requests = xQueueCreate(1, sizeof(Request)); results = xQueueCreate(1, sizeof(Result));
    if (!requests || !results) return false;
    ready = xTaskCreatePinnedToCore(task, "proposal_nn", 6144, this, 2, nullptr, 0) == pdPASS;
    runtimeReady.store(ready);
    return ready;
}

void ProposalStudent::reset() {
    ++generation; count = 0; windowStart = previousMs = resultMs = 0;
    integralCommand = correction = 0;
    for (float& value : history) value = 0;
    if (results) xQueueReset(results);
    // A running worker may finish an old request; generation rejects that result.
}

void ProposalStudent::observe(uint8_t direction, uint8_t pwm, uint32_t nowMs) {
    if (!ready || !cfg::kEnabled) return;
    if (!previousMs) { previousMs = windowStart = nowMs; return; }
    uint32_t dt = nowMs - previousMs;
    if (dt > 50U) { reset(); previousMs = windowStart = nowMs; return; }
    const float command = pwm / 255.0f * (direction == 1 ? 1.0f : direction == 2 ? -1.0f : 0.0f);
    // Previous controller output is the command held over the preceding tick.
    integralCommand += command * dt; previousMs = nowMs;
    if (nowMs - windowStart >= cfg::kIntervalMs) {
        for (int i = 0; i < 29; ++i) history[i] = history[i+1];
        history[29] = integralCommand / (nowMs - windowStart);
        integralCommand = 0; windowStart = nowMs;
        if (count < 30) ++count;
    }
}

float ProposalStudent::predict(float error, float speed, float accel, float bias, uint32_t nowMs) {
    if (!ready || !cfg::kEnabled) return 0;
    if (!std::isfinite(error) || !std::isfinite(speed) || !std::isfinite(accel) || !std::isfinite(bias) ||
        std::fabs(error) > cfg::kGuardError || std::fabs(speed) > cfg::kGuardSpeed) {
        reset(); return 0;
    }
    if (count < 30) return 0;
    Request request{};
    request.features[0] = error; request.features[1] = speed;
    request.features[2] = accel; request.features[3] = bias;
    for (int i = 0; i < 30; ++i) request.features[i+4] = history[i];
    for (int i = 0; i < 34; ++i) {
        const float value = (request.features[i]-cfg::kInputMean[i])/cfg::kInputStd[i];
        if (!std::isfinite(value) || std::fabs(value) > cfg::kGuardNormalized) { reset(); return 0; }
        request.features[i] = value;
    }
    request.stamp = nowMs; request.generation = generation;
    xQueueOverwrite(requests, &request); // Nonblocking; newest observation wins.
    Result result{};
    if (xQueueReceive(results, &result, 0) == pdTRUE && result.generation == generation) {
        correction = result.correction; resultMs = result.stamp;
    }
    if (!resultMs || nowMs-resultMs > 250U) return 0;
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
        for (int i = 0; i < 34; ++i) {
            if (std::fabs(request.features[i]*rescale) > 127.0f) { valid = false; break; }
            values[i] = dl::quantize<int8_t>(request.features[i], rescale);
        }
        float value = 0;
        if (valid) {
            model->run();
            value = dl::dequantize<int8_t>(static_cast<const int8_t*>(output->data)[0], DL_SCALE(output->exponent));
            if (!std::isfinite(value)) value = 0;
        }
        value = value < -cfg::kLimit ? -cfg::kLimit : value > cfg::kLimit ? cfg::kLimit : value;
        const Result result{value, request.stamp, request.generation};
        xQueueOverwrite(results, &result);
    }
}
