// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <linux/joystick.h>

#include <fstream>

#include "gtest/gtest.h"
#include "upkie/cpp/observation/sources/Joystick.h"

namespace upkie::cpp::observation::sources {

TEST(Joystick, WriteOnce) {
  Joystick joystick;
  Dictionary observation;
  ASSERT_NO_THROW(joystick.write(observation));
}

TEST(Joystick, UnknownEvent) {
  std::ofstream file("jsX", std::ios::binary | std::ios::out);
  Joystick joystick("jsX");

  struct js_event event;
  event.type = JS_EVENT_INIT;
  file.write(reinterpret_cast<char*>(&event), sizeof(event));
  file.flush();

  Dictionary observation;
  ASSERT_NO_THROW(joystick.write(observation));
}

TEST(Joystick, ReadButtons) {
  std::ofstream file("jsX", std::ios::binary | std::ios::out);
  Joystick joystick("jsX");

  struct js_event event;
  event.type = JS_EVENT_BUTTON;
  event.number = 0;
  event.value = 1;
  file.write(reinterpret_cast<char*>(&event), sizeof(event));
  file.flush();

  Dictionary observation;
  joystick.write(observation);
  const auto& output = observation(joystick.prefix());
  ASSERT_TRUE(output.get<bool>("cross_button"));
  ASSERT_FALSE(output.get<bool>("square_button"));
  ASSERT_FALSE(output.get<bool>("triangle_button"));

  for (uint8_t button = 2; button < 14; button++) {
    event.number = button;
    file.write(reinterpret_cast<char*>(&event), sizeof(event));
    file.flush();
    joystick.write(observation);
    ASSERT_EQ(output.get<bool>("right_button"), (button >= 5));
  }
}

TEST(Joystick, EmergencyStop) {
  std::ofstream file("jsX", std::ios::binary | std::ios::out);
  Joystick joystick("jsX");

  struct js_event event;
  event.type = JS_EVENT_BUTTON;
  event.number = 1;
  event.value = 1;
  file.write(reinterpret_cast<char*>(&event), sizeof(event));
  file.flush();

  Dictionary observation;
  ASSERT_THROW(joystick.write(observation), std::runtime_error);
}

TEST(Joystick, ReadAxes) {
  std::ofstream file("jsX", std::ios::binary | std::ios::out);
  Joystick joystick("jsX");

  struct js_event event;
  event.type = JS_EVENT_AXIS;
  event.number = 0;
  event.value = 10000;
  file.write(reinterpret_cast<char*>(&event), sizeof(event));
  file.flush();

  Dictionary observation;
  joystick.write(observation);
  const auto& output = observation(joystick.prefix());
  const Eigen::Vector2d& left_axis = output("left_axis");
  ASSERT_LE(left_axis.x(), 0.4);
  ASSERT_GE(left_axis.x(), 0.2);
  ASSERT_DOUBLE_EQ(left_axis.y(), 0.0);

  const std::vector<std::string> axes = {"left_axis", "right_axis", "pad_axis"};
  for (uint8_t axis = 1; axis < 9; axis++) {
    event.number = axis;
    file.write(reinterpret_cast<char*>(&event), sizeof(event));
    file.flush();
    joystick.write(observation);
    for (const std::string& axis : axes) {
      const Eigen::Vector2d& axis_vector = output(axis);
      ASSERT_GE(axis_vector.x(), 0.0);
      ASSERT_GE(axis_vector.y(), 0.0);
      ASSERT_LE(axis_vector.x(), 1.0);
      ASSERT_LE(axis_vector.y(), 1.0);
    }
  }
}

TEST(Joystick, ReadTriggers) {
  std::ofstream file("jsX", std::ios::binary | std::ios::out);
  Joystick joystick("jsX");
  const std::vector<std::string> triggers = {"left_trigger", "right_trigger"};
  for (uint8_t axis = 1; axis < 9; axis++) {
    struct js_event event;
    event.type = JS_EVENT_AXIS;
    event.number = axis;
    event.value = 10000;
    file.write(reinterpret_cast<char*>(&event), sizeof(event));
    file.flush();

    Dictionary observation;
    joystick.write(observation);
    for (const std::string& trigger : triggers) {
      const auto& output = observation(joystick.prefix());
      const double trigger_value = output(trigger);
      ASSERT_GE(trigger_value, -1.0);
      ASSERT_LE(trigger_value, 1.0);
    }
  }
}

TEST(Joystick, PartialInput) {
  std::ofstream file("jsX", std::ios::binary | std::ios::out);
  Joystick joystick("jsX");

  struct js_event event;
  event.type = JS_EVENT_BUTTON;
  event.number = 0;
  event.value = 1;
  file.write(reinterpret_cast<char*>(&event), sizeof(event) / 2);
  file.flush();

  // Skips the partial observation
  Dictionary observation;
  ASSERT_NO_THROW(joystick.write(observation));

  // Yet successfully reads the next event
  file.write(reinterpret_cast<char*>(&event), sizeof(event));
  file.flush();
  joystick.write(observation);
  const auto& output = observation(joystick.prefix());
  ASSERT_TRUE(output.get<bool>("cross_button"));
}

}  // namespace upkie::cpp::observation::sources
