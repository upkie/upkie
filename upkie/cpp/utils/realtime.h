// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <sched.h>
#include <sys/mman.h>

#include <stdexcept>

#ifdef __APPLE__
#include <spdlog/spdlog.h>
#endif

namespace upkie {

/*! Set the current thread to run on a given CPU core.
 *
 * \param cpu CPU core for this thread (on the Pi, CPUID in {0, 1, 2, 3}).
 *
 * \throw std::runtime_error If the operation failed.
 */
inline void configure_cpu(int cpu) {
#ifndef __APPLE__
  constexpr int kPid = 0;  // this thread
  cpu_set_t cpuset = {};
  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);
  if (::sched_setaffinity(kPid, sizeof(cpu_set_t), &cpuset) < 0) {
    throw std::runtime_error("Error setting CPU affinity");
  }
#else
  spdlog::warn("[configure_cpu] This function does nothing on macOS");
#endif
}

/*! Configure the scheduler policy to round-robin for this thread.
 *
 * \param priority Priority of this thread for the scheduler, ranging from 1
 *     (low) to 99 (high). See `man sched`. For agents, this priority should be
 *     lower than that of the spine.
 *
 * \throw std::runtime_error If the operation failed.
 */
inline void configure_scheduler(int priority) {
#ifndef __APPLE__
  constexpr int kPid = 0;  // this thread
  struct sched_param params = {};
  params.sched_priority = priority;
  if (::sched_setscheduler(kPid, SCHED_RR, &params) < 0) {
    throw std::runtime_error(
        "Error setting realtime scheduler, try running as root (use sudo)");
  }
#else
  spdlog::warn("[configure_scheduler] This function does nothing on macOS");
#endif
}

/*! Lock all memory to RAM so that the kernel doesn't page it to swap.
 *
 * The Linux man pages have a great NOTES section on this. Worth a read!
 */
inline bool lock_memory() {
  return (::mlockall(MCL_CURRENT | MCL_FUTURE) >= 0);
}

}  // namespace upkie
