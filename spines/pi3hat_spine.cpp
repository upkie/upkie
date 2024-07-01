// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023-2024 Inria

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "upkie/config/layout.h"
#include "upkie/cpp/actuation/Pi3HatInterface.h"
#include "upkie/cpp/observers/BaseOrientation.h"
#include "upkie/cpp/observers/FloorContact.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/observers/WheelOdometry.h"
#include "upkie/cpp/sensors/CpuTemperature.h"
#include "upkie/cpp/sensors/Joystick.h"
#include "upkie/cpp/spine/Spine.h"
#include "upkie/cpp/utils/get_log_path.h"
#include "upkie/cpp/utils/realtime.h"
#include "upkie/cpp/version.h"

namespace spines::pi3hat {

using Pi3Hat = ::mjbots::pi3hat::Pi3Hat;
using palimpsest::Dictionary;
using upkie::BaseOrientation;
using upkie::CpuTemperature;
using upkie::FloorContact;
using upkie::Joystick;
using upkie::ObserverPipeline;
using upkie::Pi3HatInterface;
using upkie::Spine;
using upkie::WheelOdometry;

//! Command-line arguments for the Bullet spine.
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
      } else if (arg == "--attitude-frequency") {
        attitude_frequency = std::stol(args.at(++i));
        spdlog::info("Command line: attitude_frequency = {} Hz",
                     attitude_frequency);
      } else if (arg == "--can-cpu") {
        can_cpu = std::stol(args.at(++i));
        spdlog::info("Command line: can_cpu = {}", can_cpu);
      } else if (arg == "--log-dir") {
        log_dir = args.at(++i);
        spdlog::info("Command line: log_dir = {}", log_dir);
      } else if (arg == "--shm-name") {
        shm_name = args.at(++i);
        spdlog::info("Command line: shm_name = {}", shm_name);
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
    std::cout << "--attitude-frequency <frequency>\n"
              << "    Attitude frequency in Hz.\n";
    std::cout << "--can-cpu <cpuid>\n"
              << "    CPUID for the CAN thread (default: 2).\n";
    std::cout << "--log-dir <path>\n"
              << "    Path to a directory for output logs.\n";
    std::cout << "--shm-name <name>\n"
              << "    Name for IPC shared memory file.\n";
    std::cout << "--spine-cpu <cpuid>\n"
              << "    CPUID for the spine thread (default: 1).\n";
    std::cout << "--spine-frequency <frequency>\n"
              << "    Spine frequency in Hz.\n";
    std::cout << "-v, --version\n"
              << "    Print out the spine version number.\n";
    std::cout << "\n";
  }

 public:
  //! Attitude frequency in Hz.
  unsigned attitude_frequency = 1000u;

  //! CPUID for the CAN-FD thread.
  int can_cpu = 2;

  //! Error flag.
  bool error = false;

  //! Help flag.
  bool help = false;

  //! Log directory
  std::string log_dir = "";

  //! Name for the shared memory file.
  std::string shm_name = "/vulp";

  //! CPUID for the spine thread (-1 to disable realtime).
  int spine_cpu = 1;

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;

  //! Error flag.
  bool version = false;
};

inline bool calibration_needed() {
  std::ifstream file{"/tmp/rezero_success"};
  const bool file_found = file.is_open();
  return !file_found;
}

int main(const CommandLineArguments& args) {
  if (calibration_needed()) {
    spdlog::error("Calibration needed: did you run `upkie_tool rezero`?");
    return -3;
  }
  if (!upkie::lock_memory()) {
    spdlog::error("Could not lock process memory to RAM");
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

  // Observation: Joystick
  auto joystick = std::make_shared<Joystick>();
  if (!joystick->present()) {
    char response;
    std::cout << "\n /!\\ Joystick not found /!\\\n\n"
              << "Ctrl-C will be the only way to stop the spine. "
              << "Proceed? [yN] ";
    std::cin >> response;
    if (response != 'y') {
      std::cout << "Joystick required to run the pi3hat spine.\n";
      return -6;
    }
  }
  observation.connect_sensor(joystick);

  // Observation: Floor contact
  FloorContact::Parameters floor_contact_params;
  floor_contact_params.dt = 1.0 / args.spine_frequency;
  floor_contact_params.upper_leg_joints = upkie::upper_leg_joints();
  floor_contact_params.wheels = upkie::wheel_joints();
  auto floor_contact = std::make_shared<FloorContact>(floor_contact_params);
  observation.append_observer(floor_contact);

  // Observation: Wheel odometry
  WheelOdometry::Parameters odometry_params;
  odometry_params.dt = 1.0 / args.spine_frequency;
  auto odometry = std::make_shared<WheelOdometry>(odometry_params);
  observation.append_observer(odometry);

  try {
    // pi3hat configuration
    Pi3Hat::Configuration pi3hat_config;
    pi3hat_config.attitude_rate_hz = args.attitude_frequency;
    pi3hat_config.mounting_deg.pitch = 0.;
    pi3hat_config.mounting_deg.roll = 0.;
    pi3hat_config.mounting_deg.yaw = 0.;

    // pi3hat interface
    const auto servo_layout = upkie::servo_layout();
    Pi3HatInterface interface(servo_layout, args.can_cpu, pi3hat_config);

    // Spine
    Spine::Parameters spine_params;
    spine_params.cpu = args.spine_cpu;
    spine_params.frequency = args.spine_frequency;
    spine_params.log_path = upkie::get_log_path(args.log_dir, "pi3hat_spine");
    spdlog::info("Spine data logged to {}", spine_params.log_path);
    Spine spine(spine_params, interface, observation);
    spine.run();
  } catch (const ::mjbots::pi3hat::Error& error) {
    std::string message = error.what();
    spdlog::error(message);
    if (message.find("/dev/mem") != std::string::npos) {
      spdlog::info("did you run with sudo?");
    }
    return -5;
  }

  return EXIT_SUCCESS;
}

}  // namespace spines::pi3hat

int main(int argc, char** argv) {
  spines::pi3hat::CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  } else if (args.version) {
    std::cout << "Upkie pi3hat spine " << upkie::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return spines::pi3hat::main(args);
}
