#pragma once
#include <cstdint>
// Host-test fixture only; never included by firmware CMake.
namespace proposal_model_config {
constexpr int kInputDim = 125;
constexpr int kHistory = 30;
constexpr uint32_t kIntervalMs = 100;
constexpr int kCommandOffset = 4;
constexpr int kForwardIndex = 34;
constexpr int kForwardHistoryOffset = 35;
}
