#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#pragma once
#include <c10/core/TensorImpl.h>
#include <c10/macros/Export.h>

namespace c10::impl {

class C10_API FakeTensorModeTLS {
 public:
  static void set_state(std::shared_ptr<FakeTensorMode> state);
  static void create_state(std::shared_ptr<FakeTensorMode> state);
  static void activate();
  static void deactivate();
  static std::shared_ptr<FakeTensorMode> get_state();
  static void reset_state();
};

} // namespace c10::impl

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
