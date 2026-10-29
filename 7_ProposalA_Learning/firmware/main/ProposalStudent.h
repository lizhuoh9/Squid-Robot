#pragma once
#include <cstdint>
#include "ProposalStudentConfig.h"
#include "ProposalStudentFeatures.h"
#include "dl_model_base.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

bool proposalStudentRuntimeReady();
bool residualRLRuntimeReady();

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
    float rlOutput() const { return rlApplied ? rlCorrection : 0.0f; }
    bool predictionRLApplied() const { return rlApplied; }
private:
    struct Request { float features[proposal_model_config::kInputDim]; float depth; float previousRL; uint32_t stamp; uint32_t generation; };
    struct Result { float correction; float rlCorrection; uint32_t stamp; uint32_t generation; bool valid; bool rlValid; };
    static void task(void* self);
    void run();
    void invalidateResults();
    dl::Model* model = nullptr;
    dl::TensorBase* input = nullptr;
    dl::TensorBase* output = nullptr;
    dl::Model* rlModel = nullptr;
    dl::TensorBase* rlInput = nullptr;
    dl::TensorBase* rlTensor = nullptr;
    QueueHandle_t requests = nullptr, results = nullptr;
    ProposalStudentFeatures features;
    float correction = 0;
    float rlCorrection = 0;
    uint32_t resultMs = 0, generation = 0;
    bool ready = false;
    bool resultValid = false, applied = false;
    bool rlReady = false, rlResultValid = false, rlApplied = false;
};
