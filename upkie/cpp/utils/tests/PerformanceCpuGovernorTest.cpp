// SPDX-License-Identifier: Apache-2.0

#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "upkie/cpp/utils/PerformanceCpuGovernor.h"
#include "upkie/cpp/utils/random_string.h"

namespace upkie::cpp::utils {

namespace fs = std::filesystem;

//! Fake sysfs CPU directory with four cores.
class PerformanceCpuGovernorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cpu_dir_ = fs::path(::testing::TempDir()) / ("cpu_" + random_string());
    for (int cpu = 0; cpu < 4; ++cpu) {
      fs::create_directories(governor_path(cpu).parent_path());
      write(cpu, "ondemand");
    }
  }

  void TearDown() override { fs::remove_all(cpu_dir_); }

  fs::path governor_path(int cpu) const {
    return cpu_dir_ / ("cpu" + std::to_string(cpu)) / "cpufreq" /
           "scaling_governor";
  }

  std::string read(int cpu) const {
    std::ifstream reader(governor_path(cpu));
    std::string governor;
    std::getline(reader, governor);
    return governor;
  }

  void write(int cpu, const std::string& governor) const {
    std::ofstream writer(governor_path(cpu));
    writer << governor << "\n";
  }

 protected:
  fs::path cpu_dir_;
};

TEST_F(PerformanceCpuGovernorTest, ConstructorChangesNothing) {
  {
    PerformanceCpuGovernor governor(4, cpu_dir_.string());
  }
  for (int cpu = 0; cpu < 4; ++cpu) {
    ASSERT_EQ(read(cpu), "ondemand");
  }
}

TEST_F(PerformanceCpuGovernorTest, SetsAndRestoresGovernors) {
  write(2, "schedutil");
  {
    PerformanceCpuGovernor governor(4, cpu_dir_.string());
    ASSERT_TRUE(governor.configure());
    for (int cpu = 0; cpu < 4; ++cpu) {
      ASSERT_EQ(read(cpu), "performance");
      ASSERT_EQ(governor.read_governor(cpu), "performance");
    }
  }
  ASSERT_EQ(read(0), "ondemand");
  ASSERT_EQ(read(1), "ondemand");
  ASSERT_EQ(read(2), "schedutil");
  ASSERT_EQ(read(3), "ondemand");
}

TEST_F(PerformanceCpuGovernorTest, KeepsPerformanceGovernors) {
  for (int cpu = 0; cpu < 4; ++cpu) {
    write(cpu, "performance");
  }
  {
    PerformanceCpuGovernor governor(4, cpu_dir_.string());
    ASSERT_TRUE(governor.configure());
  }
  for (int cpu = 0; cpu < 4; ++cpu) {
    ASSERT_EQ(read(cpu), "performance");
  }
}

TEST_F(PerformanceCpuGovernorTest, FailsOnMissingCpuAndRestoresOthers) {
  fs::remove_all(cpu_dir_ / "cpu3");
  {
    PerformanceCpuGovernor governor(4, cpu_dir_.string());
    ASSERT_FALSE(governor.configure());
    ASSERT_EQ(read(0), "performance");
    ASSERT_EQ(read(2), "performance");
  }
  for (int cpu = 0; cpu < 3; ++cpu) {
    ASSERT_EQ(read(cpu), "ondemand");
  }
}

}  // namespace upkie::cpp::utils
