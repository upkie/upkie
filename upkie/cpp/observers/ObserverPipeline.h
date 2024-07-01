// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <memory>
#include <vector>

#include "upkie/cpp/observers/Observer.h"
#include "upkie/cpp/observers/Source.h"

//! State observation.
namespace upkie::cpp::observation {

/*! Observer pipeline.
 *
 * An observer pipeline is a list of sources and observers, to be executed in
 * that order. Observers further down the pipeline may depend on the results of
 * those that precede them, which are written to the observation dictionary.
 * The pipeline is thus assumed to be topologically sorted.
 */
class ObserverPipeline {
  using ObserverPtrVector = std::vector<std::shared_ptr<observation::Observer>>;
  using SourcePtrVector = std::vector<std::shared_ptr<Sensor>>;
  using Dictionary = palimpsest::Dictionary;

 public:
  using iterator = ObserverPtrVector::iterator;

  /*! Reset observers.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config);

  /* Add a source at the beginning of the pipeline.
   *
   * \param source Source to append.
   *
   * \note Contrary to observers, the order in which sources are executed is
   * not guaranteed. If a source needs to run after another, consider splitting
   * it into one source and one observer.
   */
  void connect_source(std::shared_ptr<Source> source) {
    sources_.push_back(std::shared_ptr<Source>(source));
  }

  /* Append an observer at the end of the pipeline.
   *
   * \param observer Observer to append.
   */
  void append_observer(std::shared_ptr<Observer> observer) {
    observers_.push_back(std::shared_ptr<Observer>(observer));
  }

  //! Sources of the pipeline.
  SourcePtrVector& sources() { return sources_; }

  //! Number of sources in the pipeline.
  size_t nb_sources() { return sources_.size(); }

  //! Observers of the pipeline. Order matters.
  ObserverPtrVector& observers() { return observers_; }

  //! Number of observers in the pipeline.
  size_t nb_observers() { return observers_.size(); }

  /*! Run observer pipeline on an observation dictionary.
   *
   * \param[in, out] observation Observation dictionary.
   */
  void run(Dictionary& observation);

 private:
  //! Sources of the pipeline.
  SourcePtrVector sources_;

  //! Observers of the pipeline. Order matters.
  ObserverPtrVector observers_;
};

}  // namespace upkie::cpp::observation
