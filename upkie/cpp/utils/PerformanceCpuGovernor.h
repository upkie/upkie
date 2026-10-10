// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <map>
#include <string>

namespace upkie::cpp::utils {

/*! Set all CPU cores to the "performance" frequency governor while in scope.
 *
 * Latency-sensitive processes such as the pi3hat spine want all cores running
 * at maximum frequency rather than dynamically scaling. This class switches
 * CPU frequency governors to "performance" upon \ref configure, and restores
 * the governors found at startup when it is destroyed.
 *
 * Writing CPU frequency governors requires root privileges.
 */
class PerformanceCpuGovernor {
 public:
  /*! Initialize without changing any CPU governor.
   *
   * \param[in] nb_cpus Number of CPU cores to configure, with indices from 0
   *     to `nb_cpus - 1`. Defaults to the four cores of a Raspberry Pi.
   * \param[in] cpu_dir Path to the sysfs directory holding CPU devices.
   */
  explicit PerformanceCpuGovernor(
      int nb_cpus = 4, const std::string& cpu_dir = "/sys/devices/system/cpu");

  //! Restore the CPU governors found at startup.
  ~PerformanceCpuGovernor();

  // Restoring governors twice would be wrong: disable copies
  PerformanceCpuGovernor(const PerformanceCpuGovernor&) = delete;
  PerformanceCpuGovernor& operator=(const PerformanceCpuGovernor&) = delete;

  /*! Set the "performance" governor on all CPU cores.
   *
   * \return True if and only if all cores now use the "performance" governor.
   */
  bool configure();

  /*! Read the frequency governor of a CPU core.
   *
   * \param[in] cpu CPU core index.
   *
   * \return Governor name, or an empty string if it could not be read.
   */
  std::string read_governor(int cpu) const;

 private:
  /*! Path to the sysfs file holding the frequency governor of a CPU core.
   *
   * \param[in] cpu CPU core index.
   */
  std::string governor_path(int cpu) const;

  /*! Write the frequency governor of a CPU core.
   *
   * \param[in] cpu CPU core index.
   * \param[in] governor Governor name.
   *
   * \return True if and only if the governor reads back as expected.
   */
  bool write_governor(int cpu, const std::string& governor) const;

 private:
  //! Number of CPU cores to configure.
  const int nb_cpus_;

  //! Path to the sysfs directory holding CPU devices.
  const std::string cpu_dir_;

  //! Governors of the CPU cores we switched, as found at startup.
  std::map<int, std::string> initial_governors_;
};

}  // namespace upkie::cpp::utils
