#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#pragma once

#include <ATen/core/TensorBase.h>

namespace at::cuda::detail {

float *get_cublas_device_one();
float *get_cublas_device_zero();

} // namespace at::cuda::detail

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
