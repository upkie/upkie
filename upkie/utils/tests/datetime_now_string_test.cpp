// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/utils/datetime_now_string.h"

#include <string>

#include "gtest/gtest.h"

namespace upkie::utils {

TEST(DatetimeNowString, IsADatetimeString) {
  auto now = datetime_now_string();
  ASSERT_EQ(now.rfind("20", 0), 0);  // see at the turn of the next century!
  ASSERT_EQ(now[4], '-');
  ASSERT_EQ(now[7], '-');
  ASSERT_EQ(now[10], '_');
}

}  // namespace upkie::utils
