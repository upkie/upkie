// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <memory>
#include <vector>

#include "upkie/cpp/sensors/Sensor.h"

namespace upkie::cpp::sensors {

/*! Sensor pipeline.
 *
 * A sensor pipeline is a list of sensors, to be executed in that order.
 */
class SensorPipeline {
  using Dictionary = palimpsest::Dictionary;
  using Sensor = upkie::cpp::sensors::Sensor;

 public:
  /*! Add a sensor at the beginning of the pipeline.
   *
   * \param[in] sensor Sensor to append.
   *
   * \note Contrary to observers, the order in which sensors are executed is
   * not guaranteed. If a sensor needs to run after another, consider splitting
   * it into one sensor and one observer.
   */
  void connect_sensor(std::shared_ptr<Sensor> sensor) {
    sensors_.push_back(std::shared_ptr<Sensor>(sensor));
  }

  /*! Run observer pipeline on an observation dictionary.
   *
   * \param[in, out] observation Observation dictionary.
   */
  void run(Dictionary& observation);

 private:
  //! Sensors of the pipeline.
  std::vector<std::shared_ptr<Sensor>> sensors_;
};

}  // namespace upkie::cpp::sensors
