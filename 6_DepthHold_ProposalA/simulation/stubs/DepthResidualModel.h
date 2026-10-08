// Host-build stub: the ESP-DL residual model is not available on the host -> zero correction
// (identical to the firmware's behaviour when no valid model is embedded).
#ifndef SQUID_DEPTH_RESIDUAL_MODEL_STUB_H
#define SQUID_DEPTH_RESIDUAL_MODEL_STUB_H
#include <cstdint>
class DepthResidualModel {
public:
    bool begin() { return false; }
    void reset() {}
    bool ready() const { return false; }
    float predict(float, float, float, float, float, uint32_t) { return 0.0f; }
};
#endif
