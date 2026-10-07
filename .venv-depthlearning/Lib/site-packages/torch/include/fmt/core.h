#if !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
// Formatting library for C++ - core API
//
// Copyright (c) 2012 - present, Victor Zverovich and {fmt} contributors
// All rights reserved.
//
// For the license information refer to format.h.

#include "base.h"

// Using fmt::format via fmt/core.h has been deprecated since version 11
// and now requires an explicit opt in.
#ifdef FMT_DEPRECATED_HEAVY_CORE
#  include "format.h"
#endif

#else
#error "This file should not be included when either TORCH_STABLE_ONLY or TORCH_TARGET_VERSION is defined."
#endif  // !defined(TORCH_STABLE_ONLY) && !defined(TORCH_TARGET_VERSION)
