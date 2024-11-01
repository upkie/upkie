// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>
#include <spdlog/spdlog.h>

#include <Eigen/Geometry>
#include <limits>
#include <map>
#include <string>

#include "upkie/cpp/actuation/Interface.h"
#include "upkie/cpp/actuation/moteus/protocol.h"

namespace upkie::cpp::actuation {

//! Interface whose internal state perfectly tracks commands.
class MockInterface : public Interface {
 public:
  /*! Create mock actuator interface.
   *
   * \param[in] dt Simultation timestep in [s].
   */
  explicit MockInterface(const double dt);

  //! Default destructor
  ~MockInterface() = default;

  /*! Reset interface.
   *
   * \param[in] config Additional configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Write actuation-interface observations to dictionary.
   *
   * \param[out] observation Dictionary to write to.
   */
  void observe(Dictionary& observation) const override;

  /*! Process a new action dictionary.
   *
   * \param[in] action Action to read commands from.
   */
  void process_action(const Dictionary& action) override;

  /*! Simulate a new communication cycle.
   *
   * \param callback Function to call when the cycle is over.
   *
   * The callback will be invoked from an arbitrary thread when the
   * communication cycle has completed. All memory pointed to by @p data must
   * remain valid until the callback is invoked.
   */
  void cycle(std::function<void(const moteus::Output&)> callback) final;

 private:
  //! Spine timestep in [s].
  const double dt_;

  //! Mock servo replies, laid out as a servo_id -> query_result map
  std::map<int, moteus::QueryResult> query_results_;
};

}  // namespace upkie::cpp::actuation
