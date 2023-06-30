/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <vulp/actuation/Pi3HatInterface.h>
#include <vulp/observation/ObserverPipeline.h>
#include <vulp/observation/sources/CpuTemperature.h>
#include <vulp/observation/sources/Joystick.h>
#include <vulp/spine/Spine.h>
#include <vulp/utils/realtime.h>

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

#include "upkie/observers/FloorContact.h"
#include "upkie/observers/WheelOdometry.h"
#include "upkie/spines/upkie_layout.h"
#include "upkie/utils/datetime_now_string.h"

namespace upkie::spines::pi3hat {

using Pi3Hat = ::mjbots::pi3hat::Pi3Hat;
using palimpsest::Dictionary;
using upkie::observers::FloorContact;
using upkie::observers::WheelOdometry;
using vulp::actuation::Pi3HatInterface;
using vulp::observation::ObserverPipeline;
using vulp::observation::sources::CpuTemperature;
using vulp::observation::sources::Joystick;
using vulp::spine::Spine;

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
  if (!vulp::utils::lock_memory()) {
    spdlog::error("Could not lock process memory to RAM");
    return -4;
  }

  ObserverPipeline observation;

  // Observation: CPU temperature
  auto cpu_temperature = std::make_shared<CpuTemperature>();
  observation.connect_source(cpu_temperature);

  // Observation: Joystick
  auto joystick = std::make_shared<Joystick>();
  if (!joystick->present()) {
    throw std::runtime_error("Joystick required to run the pi3hat spine");
  }
  observation.connect_source(joystick);

  // Observation: Floor contact
  FloorContact::Parameters floor_contact_params;
  floor_contact_params.dt = 1.0 / args.spine_frequency;
  floor_contact_params.upper_leg_joints = upkie_layout::upper_leg_joints();
  floor_contact_params.wheels = upkie_layout::wheel_joints();
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
    const auto servo_layout = upkie_layout::servo_layout();
    Pi3HatInterface interface(servo_layout, args.can_cpu, pi3hat_config);

    // Spine
    Spine::Parameters spine_params;
    spine_params.cpu = args.spine_cpu;
    spine_params.frequency = args.spine_frequency;
    const auto now = upkie::utils::datetime_now_string();
    spine_params.log_path = args.log_dir + "/" + now + "_pi3hat_spine.mpack";
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

}  // namespace upkie::spines::pi3hat

int main(int argc, char** argv) {
  upkie::spines::pi3hat::CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  }
  return upkie::spines::pi3hat::main(args);
}
