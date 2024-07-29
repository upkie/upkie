// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/observers/observe_servos.h"

#include <palimpsest/Dictionary.h>

#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/moteus/ServoReply.h"

namespace upkie::cpp::observers {

TEST(Servo, ReadTorques) {
  palimpsest::Dictionary observation;

  std::map<int, std::string> servo_joint_map = {{0, "foo"}, {1, "bar"}};
  std::vector<actuation::moteus::ServoReply> servo_replies;
  servo_replies.push_back({1, {}});           // bar first
  servo_replies.back().result.torque = -10.;  // N.m
  servo_replies.push_back({0, {}});           // foo next
  servo_replies.back().result.torque = 10.;   // N.m

  observe_servos(observation, servo_joint_map, servo_replies);

  ASSERT_TRUE(observation.has("servo"));
  ASSERT_TRUE(observation("servo").has("foo"));
  ASSERT_TRUE(observation("servo").has("bar"));
  ASSERT_DOUBLE_EQ(observation("servo")("foo")("torque"), 10.);
  ASSERT_DOUBLE_EQ(observation("servo")("bar")("torque"), -10.);
}

}  // namespace upkie::cpp::observers
