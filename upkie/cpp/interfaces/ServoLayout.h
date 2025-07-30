// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <map>
#include <string>

#include "upkie/cpp/actuation/ServoProperties.h"

namespace upkie::cpp::actuation {

//! Map between servos, their busses and the joints they actuate.
class ServoLayout {
 public:
  /*! Add a servo to the layout.
   *
   * \param[in] servo_id Servo ID (`id.id` for moteus)
   * \param[in] bus_id CAN bus the servo is connected to.
   * \param[in] joint_name Name of the joint the servo actuates.
   * \param[in] maximum_torque Maximum torque the servo can exert, in [N m].
   */
  void add_servo(const int servo_id, const int bus_id,
                 const std::string& joint_name, const double maximum_torque) {
    servo_bus_map_[servo_id] = bus_id;
    servo_props_map_[servo_id] = ServoProperties();
    ServoProperties& props = servo_props_map_.at(servo_id);
    props.joint_name = joint_name;
    props.maximum_torque = maximum_torque;
  }

  //! Get the full servo-bus map
  const std::map<int, int>& servo_bus_map() const noexcept {
    return servo_bus_map_;
  }

  //! Get the full servo-joint map
  const std::map<int, ServoProperties>& servo_props_map() const noexcept {
    return servo_props_map_;
  }

  //! Get the number of servos in the layout.
  size_t size() const noexcept { return servo_bus_map_.size(); }

 private:
  //! Map from servo ID to the CAN bus the servo is connected to.
  std::map<int, int> servo_bus_map_;

  //! Map from servo ID to servo properties.
  std::map<int, ServoProperties> servo_props_map_;
};

}  // namespace upkie::cpp::actuation
