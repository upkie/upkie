// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/sensors/CpuTemperature.h"

namespace upkie {

CpuTemperature::CpuTemperature(const char* temp_path) : has_warned_(false) {
  fd_ = ::open(temp_path, O_RDONLY | O_NONBLOCK);
  ::memset(buffer_, 0, sizeof(buffer_));
}

CpuTemperature::~CpuTemperature() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void CpuTemperature::write(Dictionary& observation) {
  if (is_disabled_) {
    return;
  } else if (fd_ < 0) {
    spdlog::warn("CPU temperature observation disabled: file not found");
    is_disabled_ = true;
    return;
  }

  ssize_t size = ::pread(fd_, buffer_, kCpuTemperatureBufferSize, 0);
  if (size <= 0) {
    spdlog::warn("Read {} bytes from temperature file", size);
    return;
  }
  const double temperature = std::stol(buffer_) / 1000.;
  check_temperature_warning(temperature);
  auto& output = observation(prefix());
  output = temperature;
}

void CpuTemperature::check_temperature_warning(
    const double temperature) noexcept {
  constexpr double kConcerningTemperature = 75.0;
  if (temperature > kConcerningTemperature) {
    if (!has_warned_) {
      spdlog::warn("CPU temperature > {} °C, thermal throttling may occur",
                   kConcerningTemperature);
      has_warned_ = true;
    }
  }

  constexpr double kHysteresisFactor = 0.95;
  if (has_warned_ && temperature < kHysteresisFactor * kConcerningTemperature) {
    has_warned_ = false;
  }
}

}  // namespace upkie
