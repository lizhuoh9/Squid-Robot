#pragma once
#include <cstdint>
#include "ProposalStudentConfig.h"
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
    void observe(uint8_t direction, uint8_t pwm, uint32_t nowMs);
    float predict(float error, float speed, float accel, float bias, uint32_t nowMs);
private:
    struct Request { float features[34]; uint32_t stamp; uint32_t generation; };
    struct Result { float correction; uint32_t stamp; uint32_t generation; };
    static void task(void* self);
    void run();
    dl::Model* model = nullptr;
    dl::TensorBase* input = nullptr;
    dl::TensorBase* output = nullptr;
    QueueHandle_t requests = nullptr, results = nullptr;
    float history[30] = {};
    float integralCommand = 0, correction = 0;
    uint32_t windowStart = 0, previousMs = 0, resultMs = 0, generation = 0;
    int count = 0;
    bool ready = false;
};
