// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023 Inria

#pragma once

#include <memory>

#include "upkie/cpp/exceptions/UpkieError.h"
#include "upkie/cpp/sensors/CpuTemperature.h"
#include "upkie/cpp/sensors/SensorPipeline.h"

#ifndef __APPLE__
#include "upkie/cpp/sensors/Joystick.h"
#endif

namespace spines::common {

using upkie::cpp::exceptions::UpkieError;
using upkie::cpp::sensors::CpuTemperature;
using upkie::cpp::sensors::SensorPipeline;

#ifndef __APPLE__
using upkie::cpp::sensors::Joystick;
#endif

/*! Connect sensors to the observation pipeline.
 *
 * \param[in] interface Actuation interface.
 */
SensorPipeline make_sensors(bool joystick_required) {
  SensorPipeline sensors;

  // Sensor: CPU temperature
  auto cpu_temperature = std::make_shared<CpuTemperature>();
  sensors.connect_sensor(cpu_temperature);

#ifndef __APPLE__
  // Sensor: Joystick
  auto joystick = std::make_shared<Joystick>();
  if (joystick->present()) {
    spdlog::info("Joystick found");
    sensors.connect_sensor(joystick);
  } else if (joystick_required) {
    char response;
    std::cout << "\n /!\\ Joystick not found /!\\\n\n"
              << "Ctrl-C will be the only way to stop the spine. "
              << "Proceed? [yN] ";
    std::cin >> response;
    if (response != 'y' && response != 'Y') {
      throw UpkieError("Joystick required to start the spine.");
    }
  }
#endif

  return sensors;
}

}  // namespace spines::common
