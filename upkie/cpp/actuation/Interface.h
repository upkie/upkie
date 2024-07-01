// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <map>
#include <string>
#include <vector>

#include "upkie/cpp/actuation/ImuData.h"
#include "upkie/cpp/actuation/ServoLayout.h"
#include "upkie/cpp/actuation/moteus/Data.h"
#include "upkie/cpp/actuation/moteus/Output.h"
#include "upkie/cpp/actuation/resolution.h"

//! Send actions to actuators or simulators.
namespace upkie {

using palimpsest::Dictionary;

//! Base class for actuation interfaces.
class Interface {
 public:
  /*! Initialize actuation interface for a given servo layout.
   *
   * \param[in] servo_layout Servo layout.
   */
  explicit Interface(const ServoLayout& servo_layout)
      : servo_layout_(servo_layout) {
    auto query = get_query_resolution();
    auto resolution = get_position_resolution();
    for (const auto& pair : servo_layout.servo_bus_map()) {
      commands_.push_back({});
      commands_.back().id = pair.first;
      commands_.back().resolution = resolution;
      commands_.back().query = query;
    }

    replies_.resize(commands_.size());
    data_.commands = {commands_.data(), commands_.size()};
    data_.replies = {replies_.data(), replies_.size()};
  }

  //! Virtual destructor so that derived destructors are called properly.
  virtual ~Interface() = default;

  /*! Spin a new communication cycle.
   *
   * \param callback Function to call when the cycle is over.
   *
   * The callback will be invoked from an arbitrary thread when the
   * communication cycle has completed. All memory pointed to by @p data must
   * remain valid until the callback is invoked.
   */
  virtual void cycle(std::function<void(const moteus::Output&)> callback) = 0;

  /*! Reset interface using a new servo layout.
   *
   * \param[in] config Additional configuration dictionary.
   */
  virtual void reset(const Dictionary& config) = 0;

  /*! Write servo and IMU observations to dictionary.
   *
   * \param[out] observation Dictionary to write ot.
   */
  virtual void observe(Dictionary& observation) const = 0;

  //! Get servo layout.
  const ServoLayout& servo_layout() const noexcept { return servo_layout_; }

  //! Map from servo ID to the CAN bus the servo is connected to.
  const std::map<int, int>& servo_bus_map() const noexcept {
    return servo_layout_.servo_bus_map();
  }

  //! Map from servo ID to joint name.
  const std::map<int, std::string>& servo_joint_map() const noexcept {
    return servo_layout_.servo_joint_map();
  }

  //! Get servo commands.
  std::vector<moteus::ServoCommand>& commands() { return commands_; }

  //! Get servo replies.
  const std::vector<moteus::ServoReply>& replies() const { return replies_; }

  /*! Get joint command-reply data.
   *
   * This field is meant to become internal when we refactor spine servo
   * observations into actuation interfaces.
   */
  moteus::Data& data() { return data_; }

  /*! Initialize action dictionary with keys corresponding to the servo layout.
   *
   * \param[out] action Action dictionary.
   */
  void initialize_action(Dictionary& action);

  /*! Process a new action dictionary.
   *
   * \param[in] action Action to read commands from.
   */
  virtual void process_action(const Dictionary& action) = 0;

  /*! Write position commands from an action dictionary.
   *
   * \param[in] action Action to read commands from.
   */
  void write_position_commands(const Dictionary& action);

  /*! Stop all servos.
   *
   * \param[out] commands Servo commands to set to stop.
   *
   * This function does not and should not throw, as it will be called by
   * default if any exception is caught from the spine control loop.
   */
  inline void write_stop_commands() noexcept {
    for (auto& command : commands_) {
      command.mode = actuation::moteus::Mode::kStopped;
    }
  }

 protected:
  /*! Pointers to the memory shared with the CAN thread.
   *
   * This memory consists of \ref commands_ and \ref replies_. It is, by
   * definition, not thread-safe.
   */
  moteus::Data data_;

 private:
  //! Servo layout.
  ServoLayout servo_layout_;

  /*! Actuator commands.
   *
   * When running on a robot, this data is shared with the CAN thread.
   *
   * \note Not thread-safe. Seriously, not thread-safe, for real! CAN thread
   * directly reads mode and position from there. We depend on the
   * implementation of \ref Spine::cycle_actuation to guarantee that commands
   * are not modified while the CAN thread processes them.
   */
  std::vector<moteus::ServoCommand> commands_;

  /*! Servo replies shared with the CAN thread.
   *
   * \note Not thread-safe. The variable name is tedious on purpose ;)
   */
  std::vector<moteus::ServoReply> replies_;
};

}  // namespace upkie
