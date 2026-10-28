#pragma once
#include <cstdint>
#include "ProposalStudentConfig.h"
#include "ProposalStudentFeatures.h"
#include "dl_model_base.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

bool proposalStudentRuntimeReady();

// Bounded acceleration correction; no change to manual/balance/safety commands.
// Worker inference never runs while holding MotionLock on the 5ms motion task.
class ProposalStudent {
public:
    bool begin();
    void reset();
    void observe(uint8_t direction, uint8_t pwm, bool forwardActive, uint32_t nowMs,
                 bool depthValid, float depth, float speed);
    float predict(float error, float speed, float accel, float bias, uint32_t nowMs, float depth);
    bool predictionApplied() const { return applied; }
private:
    struct Request { float features[proposal_model_config::kInputDim]; uint32_t stamp; uint32_t generation; };
    struct Result { float correction; uint32_t stamp; uint32_t generation; bool valid; };
    static void task(void* self);
    void run();
    void invalidateResults();
    dl::Model* model = nullptr;
    dl::TensorBase* input = nullptr;
    dl::TensorBase* output = nullptr;
    QueueHandle_t requests = nullptr, results = nullptr;
    ProposalStudentFeatures features;
    float correction = 0;
    uint32_t resultMs = 0, generation = 0;
    bool ready = false;
    bool resultValid = false, applied = false;
};
