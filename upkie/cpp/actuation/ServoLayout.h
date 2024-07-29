// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <map>
#include <string>

namespace upkie::cpp::actuation {

//! Map between servos, their busses and the joints they actuate.
class ServoLayout {
 public:
  /*! Add a servo to the layout.
   *
   * \param[in] servo_id Servo ID (`id.id` for moteus)
   * \param[in] bus_id CAN bus the servo is connected to.
   * \param[in] joint_name Name of the joint the servo actuates.
   */
  void add_servo(const int servo_id, const int bus_id,
                 const std::string& joint_name) {
    servo_bus_map_[servo_id] = bus_id;
    servo_joint_map_[servo_id] = joint_name;
  }

  /*! Get identifier of the CAN bus a servo is connected to.
   *
   * \param[in] servo_id Servo ID.
   *
   * \return CAN bus the servo is connected to.
   *
   * \throw std::out_of_range if the servo is not in the layout.
   */
  int bus(const int servo_id) const { return servo_bus_map_.at(servo_id); }

  /*! Get the name of the joint a servo actuates.
   *
   * \param[in] servo_id Servo ID.
   *
   * \return Name of the joint the servo actuates.
   *
   * \throw std::out_of_range if the servo is not in the layout.
   */
  const std::string& joint_name(const int servo_id) const {
    return servo_joint_map_.at(servo_id);
  }

  //! Get the full servo-bus map
  const std::map<int, int>& servo_bus_map() const noexcept {
    return servo_bus_map_;
  }

  //! Get the full servo-joint map
  const std::map<int, std::string>& servo_joint_map() const noexcept {
    return servo_joint_map_;
  }

  //! Get the number of servos in the layout.
  size_t size() const noexcept { return servo_bus_map_.size(); }

 private:
  //! Map from servo ID to the CAN bus the servo is connected to.
  std::map<int, int> servo_bus_map_;

  //! Map from servo ID to joint name.
  std::map<int, std::string> servo_joint_map_;
};

}  // namespace upkie::cpp::actuation
