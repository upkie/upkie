// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <spdlog/spdlog.h>

#include <chrono>

namespace upkie {

/*! Synchronous (blocking) clock.
 *
 * This clock ticks at a fixed frequency. It provides a method to wait for the
 * next tick (which is always blocking, see below), and reports missed ticks if
 * applicable.
 *
 * The difference between a blocking clock and a rate limiter lies in the
 * behavior when skipping cycles. A rate limiter does nothing if there is no
 * time left, as the caller's rate does not need to be limited. On the
 * contrary, a synchronous clock waits for the next tick, which is by
 * definition in the future, so it always waits for a non-zero duration.
 *
 * Internally all durations in this class are stored in microseconds.
 */
class SynchronousClock {
  using microseconds = std::chrono::microseconds;

 public:
  /*! Initialize clock.
   *
   * \param frequency Desired tick frequency in [Hz]. It should be an integer
   *     that divides one million.
   */
  explicit SynchronousClock(double frequency);

  /*! Wait until the next tick of the internal clock.
   *
   * \note This function will always block and wait for the next tick into the
   * future. It will warn if it detects some missed ticks since the last call.
   */
  void wait_for_next_tick();

  //! Get measured period in seconds.
  double measured_period() const noexcept { return measured_period_; }

  //! Get last number of clock cycles skipped.
  int skip_count() const noexcept { return skip_count_; }

  //! Get the last sleep duration duration in seconds.
  double slack() const noexcept { return slack_; }

 private:
  /*! Measure period between two calls to `wait_for_next_tick`.
   *
   * \param[in] call_time Time of current call.
   */
  void measure_period(
      const std::chrono::time_point<std::chrono::steady_clock>& call_time);

 private:
  //! Desired loop duration, in microseconds.
  const microseconds period_us_;

  //! Point in time of the next clock tick.
  std::chrono::time_point<std::chrono::steady_clock> next_tick_;

  //! Time of last call to \ref wait_for_next_tick>
  std::chrono::time_point<std::chrono::steady_clock> last_call_time_;

  //! Measured period in seconds.
  double measured_period_;

  //! Number of clock cycles skipped when the tick was missed.
  int skip_count_;

  //! Last sleep duration in seconds.
  double slack_;
};

}  // namespace upkie
