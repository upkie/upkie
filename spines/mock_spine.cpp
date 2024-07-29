// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023 Inria

#include "upkie/cpp/actuation/MockInterface.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/sensors/CpuTemperature.h"

#ifndef __APPLE__
#include "upkie/cpp/sensors/Joystick.h"
#endif

#include <algorithm>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "upkie/cpp/model/joints.h"
#include "upkie/cpp/model/servo_layout.h"
#include "upkie/cpp/observers/BaseOrientation.h"
#include "upkie/cpp/observers/FloorContact.h"
#include "upkie/cpp/observers/WheelOdometry.h"
#include "upkie/cpp/spine/Spine.h"
#include "upkie/cpp/utils/get_log_path.h"
#include "upkie/cpp/utils/realtime.h"
#include "upkie/cpp/version.h"

namespace spines::mock {

using palimpsest::Dictionary;
using upkie::cpp::actuation::MockInterface;
using upkie::cpp::observers::BaseOrientation;
using upkie::cpp::observers::FloorContact;
using upkie::cpp::observers::ObserverPipeline;
using upkie::cpp::observers::WheelOdometry;
using upkie::cpp::sensors::CpuTemperature;
using upkie::cpp::spine::Spine;

#ifndef __APPLE__
using upkie::cpp::sensors::Joystick;
#endif

//! Command-line arguments for the mock spine.
class CommandLineArguments {
 public:
  /*! Read command line arguments.
   *
   * \param[in] args List of command-line arguments.
   */
  explicit CommandLineArguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "-v" || arg == "--version") {
        version = true;
      } else if (arg == "--log-dir") {
        log_dir = args.at(++i);
        spdlog::info("Command line: log_dir = {}", log_dir);
      } else if (arg == "--spine-cpu") {
        spine_cpu = std::stol(args.at(++i));
        spdlog::info("Command line: spine_cpu = {}", spine_cpu);
      } else if (arg == "--spine-frequency") {
        spine_frequency = std::stol(args.at(++i));
        spdlog::info("Command line: spine_frequency = {} Hz", spine_frequency);
      } else {
        spdlog::error("Unknown argument: {}", arg);
        error = true;
      }
    }
    if (log_dir.length() < 1) {
      const char* env_log_dir = std::getenv("UPKIE_LOG_PATH");
      log_dir = (env_log_dir != nullptr) ? env_log_dir : "/tmp";
    }
  }

  /*! Show help message
   *
   * \param[in] name Binary name from argv[0].
   */
  inline void print_usage(const char* name) noexcept {
    std::cout << "Usage: " << name << " [options]\n";
    std::cout << "\n";
    std::cout << "Optional arguments:\n\n";
    std::cout << "-h, --help\n"
              << "    Print this help and exit.\n";
    std::cout << "--spine-cpu <cpuid>\n"
              << "    CPUID for the spine thread (default: -1).\n";
    std::cout << "--spine-frequency <frequency>\n"
              << "    Spine frequency in Hertz.\n";
    std::cout << "-v, --version\n"
              << "    Print out the spine version number.\n";
    std::cout << "\n";
  }

 public:
  //! Error flag
  bool error = false;

  //! Help flag
  bool help = false;

  //! Log directory
  std::string log_dir = "";

  //! CPUID for the spine thread (-1 to disable realtime).
  int spine_cpu = -1;

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;

  //! Version flag
  bool version = false;
};

int main(const CommandLineArguments& args) {
  if (!upkie::cpp::utils::lock_memory()) {
    spdlog::error("could not lock process memory to RAM");
    return -4;
  }

  ObserverPipeline observation;

  // Observation: Base orientation
  BaseOrientation::Parameters base_orientation_params;
  auto base_orientation =
      std::make_shared<BaseOrientation>(base_orientation_params);
  observation.append_observer(base_orientation);

  // Observation: CPU temperature
  auto cpu_temperature = std::make_shared<CpuTemperature>();
  observation.connect_sensor(cpu_temperature);

#ifndef __APPLE__
  // Observation: Joystick
  auto joystick = std::make_shared<Joystick>();
  if (joystick->present()) {
    spdlog::info("Joystick found");
    observation.connect_sensor(joystick);
  }
#endif

  // Observation: Floor contact
  FloorContact::Parameters floor_contact_params;
  floor_contact_params.dt = 1.0 / args.spine_frequency;
  floor_contact_params.upper_leg_joints = upkie::cpp::model::upper_leg_joints();
  floor_contact_params.wheels = upkie::cpp::model::wheel_joints();
  auto floor_contact = std::make_shared<FloorContact>(floor_contact_params);
  observation.append_observer(floor_contact);

  // Observation: Wheel odometry
  WheelOdometry::Parameters odometry_params;
  odometry_params.dt = 1.0 / args.spine_frequency;
  auto odometry = std::make_shared<WheelOdometry>(odometry_params);
  observation.append_observer(odometry);

  // Mock actuators
  const auto servo_layout = upkie::cpp::model::servo_layout();
  const double dt = 1.0 / args.spine_frequency;
  MockInterface actuation(servo_layout, dt);

  // Spine
  Spine::Parameters spine_params;
  spine_params.cpu = args.spine_cpu;
  spine_params.frequency = args.spine_frequency;
  spine_params.log_path =
      upkie::cpp::utils::get_log_path(args.log_dir, "mock_spine");
  spdlog::info("Spine data logged to {}", spine_params.log_path);
  Spine spine(spine_params, actuation, observation);
  spine.run();

  return EXIT_SUCCESS;
}

}  // namespace spines::mock

int main(int argc, char** argv) {
  spines::mock::CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  } else if (args.version) {
    std::cout << "Upkie mock spine " << upkie::cpp::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return spines::mock::main(args);
}
