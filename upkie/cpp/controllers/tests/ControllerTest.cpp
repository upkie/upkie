// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/controllers/Controller.h"

namespace upkie::cpp::controllers {

TEST(Controller, BaseClassControlsNothing) {
  Controller controller;
  Dictionary observation, action;
  ASSERT_TRUE(observation.is_empty());
  ASSERT_TRUE(action.is_empty());
  controller.read(observation, action);
  controller.write(action);
  ASSERT_TRUE(action.is_empty());
}

}  // namespace upkie::cpp::controllers
