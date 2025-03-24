// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <unistd.h>

#include <string>

#include "upkie/cpp/sensors/Sensor.h"

namespace upkie::cpp::sensors {

//! Deadband between 0.0 and 1.0.
constexpr double kJoystickDeadband = 0.1;

/*! Sensor for a joystick controller.
 *
 * \note This sensor only works on Linux.
 *
 * Axes are the same for PS4 and Xbox controllers, but buttons differ slightly.
 *
 * ## Outputs
 *
 * This sensor has the following outputs:
 *
 * ### Buttons
 *
 * | Key | Type | Description |
 * |-----|------|-------------|
 * | `cross_button` | float | bottom button (cross on PS4, A on Xbox) |
 * | `square_button` | float | left button (square on PS4, Y on Xbox) |
 * | `triangle_button` | float | top button (triangle on PS4, X on Xbox) |
 * | `left_button` | float | L1 on PS4, L on Xbox |
 * | `right_button` | float | R1 on PS4, R on Xbox |
 *
 * Note that the **red button** (circle on PS4, B on Xbox) serves as **emergency
 * stop** 🚨: when it is pressed, the spine will terminate.
 *
 * ### Continus axes
 *
 * | Key | Type | Description |
 * |-----|------|-------------|
 * | `left_axis` | [float, float] | left analog joystick |
 * | `right_axis` | [float, float] | right analog joystick |
 * | `left_trigger` | float | left trigger if there is one (L2 on PS4) |
 * | `right_trigger` | float | right trigger if there is one (R2 on PS4) |
 *
 * ### Discrete axes
 *
 * | Key | Type | Description |
 * |-----|------|-------------|
 * | `pad_axis` | [float, float] | directional pad |
 *
 */
class Joystick : public Sensor {
 public:
  /*! Open the device file.
   *
   * \param[in] device_path Path to the joystick device file.
   */
  Joystick(const std::string& device_path = "/dev/input/js0");

  //! Close device file.
  ~Joystick() override;

  //! Check if the device file was opened successfully
  inline bool present() const noexcept { return (fd_ >= 0); }

  //! Prefix of output in the observation dictionary.
  inline std::string prefix() const noexcept final { return "joystick"; }

  /*! Write output to a dictionary.
   *
   * \param[out] output Dictionary to write observations to.
   */
  void write(Dictionary& output) final;

 private:
  //! Read next joystick event from the device file.
  void read_event();

 private:
  //! File descriptor to the kernel virtual file
  int fd_;

  //! Joystick event
  struct js_event event_;

  //! Left axis coordinates between -1.0 and 1.0
  Eigen::Vector2d left_axis_ = Eigen::Vector2d::Zero();

  //! Left trigger position between -1.0 and 1.0
  double left_trigger_ = -1.0;

  //! Right axis coordinates between -1.0 and 1.0
  Eigen::Vector2d right_axis_ = Eigen::Vector2d::Zero();

  //! Right trigger position between -1.0 and 1.0
  double right_trigger_ = -1.0;

  //! Pad axis coordinates between -1.0 and 1.0
  Eigen::Vector2d pad_axis_ = Eigen::Vector2d::Zero();

  //! Cross button
  bool cross_button_ = false;

  //! Left button
  bool left_button_ = false;

  //! Right button
  bool right_button_ = false;

  //! Square button
  bool square_button_ = false;

  //! Triangle button
  bool triangle_button_ = false;
};

}  // namespace upkie::cpp::sensors
