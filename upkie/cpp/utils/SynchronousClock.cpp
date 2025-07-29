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

namespace upkie::cpp::utils {

SynchronousClock::SynchronousClock(double frequency)
    : period_us_(microseconds(static_cast<int64_t>(1e6 / frequency))),
      measured_period_(1.0 / frequency),
      skip_count_(0),
      slack_(0.0),
      frequency_(frequency),
      accumulated_skip_count_(0) {
  assert(divides(1000000u, static_cast<unsigned>(frequency)));
  const auto now = std::chrono::steady_clock::now();
  last_call_time_ = now;
  last_skip_warning_time_ = now;
  measured_period_ = 1. / frequency;
  next_tick_ = now + period_us_;
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
    accumulated_skip_count_ += skip_count_;
    const auto skip_warning_time = std::chrono::steady_clock::now();
    const auto time_since_last_warning =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            skip_warning_time - last_skip_warning_time_)
            .count() /
        1000.0;
    if (time_since_last_warning >= kSkipWarningInterval_) {
      const double expected_steps = time_since_last_warning * frequency_;
      const double skip_ratio = accumulated_skip_count_ / expected_steps;
      const double estimated_frequency = (1.0 - skip_ratio) * frequency_;
      spdlog::warn(
          "Skipped {} clock cycles over {:.1f} seconds (frequency is at most "
          "{:.0f} Hz < desired {:.0f} Hz)",
          accumulated_skip_count_, time_since_last_warning, estimated_frequency,
          frequency_);
      last_skip_warning_time_ = skip_warning_time;
      accumulated_skip_count_ = 0;
    }
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

}  // namespace upkie::cpp::utils
