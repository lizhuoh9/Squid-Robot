#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#include <Python.h>

namespace torch::functorch::impl {

void initFuncTorchBindings(PyObject* module);
PyObject* unwrap_dead_wrappers(PyObject* args);

} // namespace torch::functorch::impl

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
