// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

namespace upkie::observers {

using palimpsest::exceptions::KeyError;
using vulp::utils::low_pass_filter;

BaseOrientation::BaseOrientation(const Parameters& params) : params_(params) {}

}  // namespace upkie::observers
