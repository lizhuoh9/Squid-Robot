#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#pragma once

// Replacement values are computed per-dtype on the host (CUDA-style: the
// posinf/neginf defaults are the input dtype's max/lowest) and ride at float,
// which represents half/bfloat extrema exactly; the struct is always
// instantiated at T = float.
template <typename T>
struct NanToNumParams {
  T nan;
  T posinf;
  T neginf;
};

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
