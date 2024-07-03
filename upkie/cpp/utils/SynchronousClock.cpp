// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     moteus_control_example.cc from github.com:mjbots/pi3hat
 *     SPDX-License-Identifier: Apache-2.0
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 */

#include "upkie/cpp/utils/SynchronousClock.h"

#include <thread>

#include "upkie/cpp/utils/math.h"

namespace upkie::utils {

SynchronousClock::SynchronousClock(double frequency)
    : period_us_(microseconds(static_cast<int64_t>(1e6 / frequency))),
      measured_period_(1.0 / frequency),
      skip_count_(0),
      slack_(0.0) {
  assert(math::divides(1000000u, static_cast<unsigned>(frequency)));
  last_call_time_ = std::chrono::steady_clock::now();
  measured_period_ = 1. / frequency;
  next_tick_ = std::chrono::steady_clock::now() + period_us_;
}

void SynchronousClock::measure_period(
    const std::chrono::time_point<std::chrono::steady_clock>& call_time) {
  const auto measured_period = call_time - last_call_time_;
  measured_period_ =
      std::chrono::duration_cast<microseconds>(measured_period).count() / 1e6;
  last_call_time_ = call_time;
}

void SynchronousClock::wait_for_next_tick() {
  const auto call_time = std::chrono::steady_clock::now();
  const double duration_tick_to_call_us =
      std::chrono::duration_cast<microseconds>(call_time - next_tick_).count();
  skip_count_ = std::ceil(duration_tick_to_call_us / period_us_.count());
  if (skip_count_ > 0) {
    next_tick_ += skip_count_ * period_us_;
  }
  std::this_thread::sleep_until(next_tick_);
  if (skip_count_ > 0) {
    spdlog::warn("Skipped {} clock cycles", skip_count_);
    slack_ = 0.0;
  } else {
    const auto wakeup_time = std::chrono::steady_clock::now();
    const auto sleep_duration = wakeup_time - call_time;
    const double duration_call_to_wakeup_us =
        std::chrono::duration_cast<microseconds>(sleep_duration).count();
    slack_ = duration_call_to_wakeup_us / 1e6;
  }
  next_tick_ += period_us_;
  measure_period(call_time);
}

}  // namespace upkie::utils
