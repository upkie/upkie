// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <map>
#include <string>
#include <vector>

#include "upkie/cpp/actuation/ImuData.h"
#include "upkie/cpp/actuation/ServoLayout.h"
#include "upkie/cpp/actuation/moteus/Data.h"
#include "upkie/cpp/actuation/moteus/Output.h"
#include "upkie/cpp/actuation/static_config.h"

//! Send actions to actuators or simulators.
namespace upkie::cpp::actuation {

using palimpsest::Dictionary;

//! Base class for actuation interfaces.
class Interface {
 public:
  //! Initialize actuation interface for a given servo layout.
  Interface();

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

  /*! Reset interface.
   *
   * \param[in] config New configuration dictionary.
   */
  virtual void reset(const Dictionary& config) = 0;

  /*! Write servo and IMU observations to dictionary.
   *
   * \param[out] observation Dictionary to write to.
   */
  virtual void observe(Dictionary& observation) const = 0;

  //! Get servo layout.
  const ServoLayout& servo_layout() const noexcept { return servo_layout_; }

  //! Map from servo ID to the CAN bus the servo is connected to.
  const std::map<int, int>& servo_bus_map() const noexcept {
    return servo_layout_.servo_bus_map();
  }

  //! Map from servo ID to joint name.
  const std::map<int, std::string>& servo_name_map() const noexcept {
    return servo_name_map_;
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

  /*! Reset action dictionary to the default action.
   *
   * \param[out] action Action dictionary.
   */
  void reset_action(Dictionary& action);

  /*! Process a new action dictionary.
   *
   * \param[in] action Action to read commands from.
   */
  virtual void process_action(const Dictionary& action) = 0;

  /*! Write position commands from an action dictionary.
   *
   * \param[in] action Action to read commands from.
   * \throw PositionCommandError If any position command is incorrect.
   */
  void write_position_commands(const Dictionary& action);

  /*! Stop all servos.
   *
   * This function does not and should not throw, as it will be called by
   * default if any exception is caught from the spine control loop.
   */
  inline void write_stop_commands() noexcept {
    for (auto& command : commands_) {
      command.mode = moteus::Mode::kStopped;
    }
  }

  /*! Observe IMU measurements.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  inline void observe_imu(Dictionary& observation) const {
    observation("imu")("orientation") = imu_data_.orientation_imu_in_ars;
    observation("imu")("angular_velocity") =
        imu_data_.angular_velocity_imu_in_imu;
    observation("imu")("linear_acceleration") =
        imu_data_.linear_acceleration_imu_in_imu;
    observation("imu")("raw_angular_velocity") = imu_data_.raw_angular_velocity;
    observation("imu")("raw_linear_acceleration") =
        imu_data_.raw_linear_acceleration;
  }

 protected:
  /*! Pointers to the memory shared with the CAN thread.
   *
   * This memory consists of commands and replies. It is, by definition, not
   * thread-safe.
   */
  moteus::Data data_;

  //! IMU data.
  ImuData imu_data_;

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

  //! Map from servo ID to joint name.
  std::map<int, std::string> servo_name_map_;
};

}  // namespace upkie::cpp::actuation
