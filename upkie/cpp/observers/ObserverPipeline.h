// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <memory>
#include <vector>

#include "upkie/cpp/observers/Observer.h"

//! State observation.
namespace upkie::cpp::observers {

/*! Observer pipeline.
 *
 * An observer pipeline is a list of observers, to be executed in that order.
 * Observers further down the pipeline may depend on the results of those that
 * precede them, which are written to the observation dictionary. The pipeline
 * is thus assumed to be topologically sorted.
 */
class ObserverPipeline {
  using Dictionary = palimpsest::Dictionary;

 public:
  /*! Reset observers.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config);

  /*! Append an observer at the end of the pipeline.
   *
   * \param observer Observer to append.
   */
  void append_observer(std::shared_ptr<Observer> observer) {
    observers_.push_back(std::shared_ptr<Observer>(observer));
  }

  //! Observers of the pipeline. Order matters.
  std::vector<std::shared_ptr<Observer>>& observers() { return observers_; }

  /*! Run observer pipeline on an observation dictionary.
   *
   * \param[in, out] observation Observation dictionary.
   */
  void run(Dictionary& observation);

 private:
  //! Observers of the pipeline. Order matters.
  std::vector<std::shared_ptr<Observer>> observers_;
};

}  // namespace upkie::cpp::observers
