#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#pragma once

#if defined(USE_ROCM)
#include <thrust/distance.h>
#include <thrust/functional.h>
namespace TORCH_CUDA_STD_NS = ::thrust;
#else
#include <cuda/std/functional>
#include <cuda/std/iterator>
namespace TORCH_CUDA_STD_NS = ::cuda::std;
#endif

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
