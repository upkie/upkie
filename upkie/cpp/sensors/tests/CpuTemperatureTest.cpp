// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "gtest/gtest.h"
#include "upkie/cpp/sensors/CpuTemperature.h"

namespace upkie::cpp::sensors {

TEST(CpuTemperature, NoWriteIfDisabled) {
  CpuTemperature cpu_temperature("/bogus/temp");
  Dictionary observation;
  ASSERT_FALSE(cpu_temperature.is_disabled());
  ASSERT_NO_THROW(cpu_temperature.write(observation));
  ASSERT_TRUE(cpu_temperature.is_disabled());
}

TEST(CpuTemperature, WriteOnce) {
  CpuTemperature cpu_temperature;
  Dictionary observation;
  ASSERT_NO_THROW(cpu_temperature.write(observation));

  // Write may have failed, e.g. in GitHub Actions
  if (observation.has(cpu_temperature.prefix())) {
    double temperature = observation(cpu_temperature.prefix());
    ASSERT_GT(temperature, 0.0);
    ASSERT_LT(temperature, 100.0);
  }
}

}  // namespace upkie::cpp::sensors
