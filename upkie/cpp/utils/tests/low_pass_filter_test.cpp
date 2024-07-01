// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2024 Inria

#include "upkie/cpp/utils/low_pass_filter.h"

#include "gtest/gtest.h"
#include "upkie/cpp/exceptions/FilterError.h"

namespace upkie::cpp::utils {

TEST(InlineLowPassFilter, InformationLoss) {
  const double prev_output = 0.0;
  const double dt = 1.23456789;
  const double input = 5.0;
  ASSERT_THROW(low_pass_filter(prev_output, dt, input, dt), FilterError);
}

TEST(InlineLowPassFilter, Converges) {
  const double target = 1.0;
  const double dt = 1e-3;
  const double T = 10 * dt;
  double output = 0.0;
  for (unsigned i = 0; i < 20; i++) {
    output = low_pass_filter(output, T, target, dt);
  }
  ASSERT_GT(output, 0.8);
  ASSERT_LT(output, target);
}

}  // namespace upkie::cpp::utils
