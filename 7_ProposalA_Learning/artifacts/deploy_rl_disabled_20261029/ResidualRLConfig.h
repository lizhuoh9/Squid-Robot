// RL correction deliberately disabled; deployment uses the known NN +/-0.5 model.
// Generated experimental RL metadata; acceptance is reported separately.
#pragma once
namespace residual_rl_config {
constexpr bool kEnabled = false;
constexpr int kInputDim = 128;
constexpr float kLimit = 0.8f;
constexpr float kInputScale = 0.0625f;
constexpr float kLogitScale = 0.0625f;
constexpr char kBaseCheckpointSha256[] = "b60015f2d2e272c882c572c85787dc43eeb402694d737d01fe1b9cd9896312f8";
constexpr char kActorSha256[] = "e1ffe6e0597960055cb48dc6cba9cc0a3b7658d54e567ec754ae006e0c7d3a9f";
}
