// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <memory>
#include <vector>

#include "upkie/cpp/observers/Observer.h"
#include "upkie/cpp/sensors/Sensor.h"

//! State observation.
namespace upkie::cpp::observers {

/*! Observer pipeline.
 *
 * An observer pipeline is a list of sensors and observers, to be executed in
 * that order. Observers further down the pipeline may depend on the results of
 * those that precede them, which are written to the observation dictionary.
 * The pipeline is thus assumed to be topologically sorted.
 */
class ObserverPipeline {
  using Dictionary = palimpsest::Dictionary;
  using Sensor = upkie::cpp::sensors::Sensor;

 public:
  /*! Reset observers.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config);

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

  /*! Append an observer at the end of the pipeline.
   *
   * \param observer Observer to append.
   */
  void append_observer(std::shared_ptr<Observer> observer) {
    observers_.push_back(std::shared_ptr<Observer>(observer));
  }

  //! Sensors of the pipeline.
  std::vector<std::shared_ptr<Sensor>>& sensors() { return sensors_; }

  //! Observers of the pipeline. Order matters.
  std::vector<std::shared_ptr<Observer>>& observers() { return observers_; }

  /*! Run observer pipeline on an observation dictionary.
   *
   * \param[in, out] observation Observation dictionary.
   */
  void run(Dictionary& observation);

 private:
  //! Sensors of the pipeline.
  std::vector<std::shared_ptr<Sensor>> sensors_;

  //! Observers of the pipeline. Order matters.
  std::vector<std::shared_ptr<Observer>> observers_;
};

}  // namespace upkie::cpp::observers
