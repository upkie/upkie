// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#include "upkie/cpp/sensors/Sensor.h"

#include <palimpsest/Dictionary.h>

#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"

using palimpsest::Dictionary;

namespace upkie::cpp::sensors {

TEST(Sensor, UnknownPrefix) {
  Sensor sensor;
  ASSERT_EQ(sensor.prefix().rfind("unknown", 0), 0);
}

TEST(Sensor, WritesNothing) {
  Sensor sensor;
  Dictionary observation;
  sensor.write(observation);
  ASSERT_TRUE(observation.is_empty());
}

}  // namespace upkie::cpp::sensors
