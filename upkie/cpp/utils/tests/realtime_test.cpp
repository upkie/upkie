// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/utils/realtime.h"

#include "gtest/gtest.h"

namespace upkie::cpp::utils {

TEST(Realtime, ConfigureCPU) { ASSERT_NO_THROW(configure_cpu(0)); }

TEST(Realtime, ConfigureScheduler) {
  constexpr int priority = 10;
  ASSERT_THROW(configure_scheduler(priority), std::runtime_error);
}

}  // namespace upkie::cpp::utils
