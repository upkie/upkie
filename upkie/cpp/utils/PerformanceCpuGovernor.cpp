// SPDX-License-Identifier: Apache-2.0

#include "upkie/cpp/utils/PerformanceCpuGovernor.h"

#include <spdlog/spdlog.h>

#include <fstream>

namespace upkie::cpp::utils {

PerformanceCpuGovernor::PerformanceCpuGovernor(int nb_cpus,
                                               const std::string& cpu_dir)
    : nb_cpus_(nb_cpus), cpu_dir_(cpu_dir) {}

PerformanceCpuGovernor::~PerformanceCpuGovernor() {
  for (const auto& [cpu, governor] : initial_governors_) {
    if (write_governor(cpu, governor)) {
      spdlog::info("Restored CPU {} frequency governor to \"{}\"", cpu,
                   governor);
    } else {
      spdlog::warn("Could not restore CPU {} frequency governor to \"{}\"", cpu,
                   governor);
    }
  }
}

bool PerformanceCpuGovernor::configure() {
  for (int cpu = 0; cpu < nb_cpus_; ++cpu) {
    const std::string governor = read_governor(cpu);
    if (governor.empty()) {
      spdlog::error("Could not read CPU {} frequency governor", cpu);
      return false;
    } else if (governor == "performance") {
      spdlog::info("CPU {} frequency governor is already on \"performance\"",
                   cpu, governor);
      continue;
    }
    spdlog::info(
        "Setting CPU {} frequency governor from \"{}\" to \"performance\"", cpu,
        governor);
    if (!write_governor(cpu, "performance")) {
      spdlog::error(
          "Could not set CPU {} frequency governor to \"performance\"", cpu);
      return false;
    }
    initial_governors_[cpu] = governor;
  }
  return true;
}

std::string PerformanceCpuGovernor::read_governor(int cpu) const {
  std::ifstream reader(governor_path(cpu));
  std::string governor;
  std::getline(reader, governor);
  return governor;
}

std::string PerformanceCpuGovernor::governor_path(int cpu) const {
  return cpu_dir_ + "/cpu" + std::to_string(cpu) + "/cpufreq/scaling_governor";
}

bool PerformanceCpuGovernor::write_governor(int cpu,
                                            const std::string& governor) const {
  {
    std::ofstream writer(governor_path(cpu));
    writer << governor << std::flush;
    if (!writer.good()) {
      return false;
    }
  }
  return read_governor(cpu) == governor;
}

}  // namespace upkie::cpp::utils
