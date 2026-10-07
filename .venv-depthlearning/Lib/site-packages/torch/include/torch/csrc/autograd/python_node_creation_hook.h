#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
#pragma once

#include <c10/core/SafePyObject.h>
#include <torch/csrc/autograd/node_creation_hook.h>

#include <memory>
#include <vector>

namespace torch::autograd {

struct PyNodeCreationHook : public NodeCreationHook {
  explicit PyNodeCreationHook(c10::SafePyObject hook);
  void operator()(const c10::intrusive_ptr<Node>& node) override;

 private:
  c10::SafePyObject hook_;
};

struct PyDefaultNodeCreationHooks {
  // Wraps the node creation hooks currently registered in the ATen TLS,
  // outermost first.
  static std::vector<std::unique_ptr<NodeCreationHook>> get_hooks();
};

} // namespace torch::autograd

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
