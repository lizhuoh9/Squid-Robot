#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#pragma once

#include <torch/csrc/Stream.h>
#include <torch/csrc/python_headers.h>

namespace at::mps {
class MPSStream;
} // namespace at::mps

// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
struct THPMPSStream : THPStream {
  // Non-owning pointer to one of the streams in the MPS stream pool
  at::mps::MPSStream* mps_stream;
};
extern PyObject* THPMPSStreamClass;

void THPMPSStream_init(PyObject* module);

inline bool THPMPSStream_Check(PyObject* obj) {
  return THPMPSStreamClass && PyObject_IsInstance(obj, THPMPSStreamClass);
}

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
