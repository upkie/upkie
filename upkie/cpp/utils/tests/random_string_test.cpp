// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/utils/random_string.h"

#include <string>

#include "gtest/gtest.h"

namespace upkie {

TEST(RandomString, HasCorrectLength) {
  ASSERT_EQ(random_string(16).size(), 16);
  ASSERT_EQ(random_string(32).size(), 32);
}

TEST(RandomString, TrimmedAtSixtyTwo) {
  ASSERT_EQ(random_string(62).size(), 62);
  ASSERT_EQ(random_string(63).size(), 62);
  ASSERT_EQ(random_string(123456).size(), 62);
}

TEST(RandomString, DoesNotRepeat) {
  std::string first_string = random_string();
  std::string second_string = random_string();
  ASSERT_NE(first_string, second_string);
}

}  // namespace upkie
