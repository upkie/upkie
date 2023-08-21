/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <vulp/actuation/ServoLayout.h>

#include <string>
#include <vector>

namespace upkie::config {

/*! Get Upkie's servo layout.
 *
 * \return Upkie's servo layout.
 */
inline const vulp::actuation::ServoLayout servo_layout() noexcept {
  vulp::actuation::ServoLayout layout;
  layout.add_servo(1, 1, "left_hip");
  layout.add_servo(2, 1, "left_knee");
  layout.add_servo(3, 1, "left_wheel");
  layout.add_servo(4, 2, "right_hip");
  layout.add_servo(5, 2, "right_knee");
  layout.add_servo(6, 2, "right_wheel");
  return layout;
}

/*! Get list of upper leg joints, i.e. hips and knees.
 *
 * \return Vector of upper leg joint names.
 */
inline const std::vector<std::string> upper_leg_joints() noexcept {
  return {"left_hip", "left_knee", "right_hip", "right_knee"};
}

/*! Get list of wheel joints.
 *
 * \return Vector of wheel joint names.
 */
inline const std::vector<std::string> wheel_joints() noexcept {
  return {"left_wheel", "right_wheel"};
}

}  // namespace upkie::config
