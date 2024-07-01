// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>

#include "upkie/cpp/observers/Source.h"

namespace upkie::cpp::observation::sources {

//! Characters required to read the temperature in [mC] from the kernel.
constexpr unsigned kCpuTemperatureBufferSize = 12;

/*! Source for CPU temperature readings.
 *
 * \note This source only works on Linux.
 */
class CpuTemperature : public Source {
 public:
  /*! Open file to query temperature from the kernel.
   *
   * \param[in] temp_path Path to thermal-zone special file from the Linux
   * kernel.
   */
  CpuTemperature(
      const char* temp_path = "/sys/class/thermal/thermal_zone0/temp");

  //! Close file.
  ~CpuTemperature() override;

  //! Prefix of output in the observation dictionary.
  inline std::string prefix() const noexcept final { return "cpu_temperature"; }

  /*! Write output to a dictionary.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final;

  //! Check if temperature observations are disabled.
  bool is_disabled() const { return is_disabled_; }

 private:
  /*! Issue a warning if the temperature is too high.
   *
   * \param[in] temperature Current temperature in °C.
   *
   * This warning is rather specific to the Raspberry Pi.
   */
  void check_temperature_warning(const double temperature) noexcept;

 private:
  //! Write function does nothing if this flag is set.
  bool is_disabled_ = false;

  //! File descriptor to the kernel virtual file.
  int fd_;

  //! Buffer to hold the temperature string read from file.
  char buffer_[kCpuTemperatureBufferSize];

  //! Flag set to true when issuing a temperature warning.
  bool has_warned_;
};

}  // namespace upkie::cpp::observation::sources
