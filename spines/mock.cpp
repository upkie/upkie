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

#include <mpacklog/Logger.h>
#include <vulp/actuation/MockInterface.h>
#include <vulp/observation/ObserverPipeline.h>
#include <vulp/observation/sources/CpuTemperature.h>
#include <vulp/observation/sources/Joystick.h>
#include <vulp/spine/Spine.h>
#include <vulp/utils/realtime.h>

#include <algorithm>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "upkie_locomotion/observers/FloorContact.h"
#include "upkie_locomotion/observers/WheelOdometry.h"
#include "upkie_locomotion/spines/upkie.h"

namespace upkie_locomotion::spines::mock {

using mpacklog::Logger;
using palimpsest::Dictionary;
using upkie_locomotion::observers::FloorContact;
using upkie_locomotion::observers::WheelOdometry;
using vulp::actuation::MockInterface;
using vulp::observation::ObserverPipeline;
using vulp::observation::sources::CpuTemperature;
using vulp::observation::sources::Joystick;
using vulp::spine::Spine;

namespace upkie = upkie_locomotion::spines::upkie;

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
    std::cout << "\n";
  }

 public:
  //! Error flag
  bool error = false;

  //! Help flag
  bool help = false;

  //! CPUID for the spine thread (-1 to disable realtime).
  int spine_cpu = -1;

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;
};

int main(const CommandLineArguments& args) {
  if (!vulp::utils::lock_memory()) {
    spdlog::error("could not lock process memory to RAM");
    return -4;
  }

  ObserverPipeline observation;

  // Observation: CPU temperature
  auto cpu_temperature = std::make_shared<CpuTemperature>();
  observation.connect_source(cpu_temperature);

  // Observation: Joystick
  auto joystick = std::make_shared<Joystick>();
  if (joystick->present()) {
    spdlog::info("Joystick found");
    observation.connect_source(joystick);
  }

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

  // Mock actuators
  const auto servo_layout = upkie::servo_layout();
  const double dt = 1.0 / args.spine_frequency;
  MockInterface actuation(servo_layout, dt);

  // Spine
  Spine::Parameters spine_params;
  spine_params.cpu = args.spine_cpu;
  spine_params.frequency = args.spine_frequency;
  spine_params.log_path = "/dev/shm/spine.mpack";
  Spine spine(spine_params, actuation, observation);
  spine.run();

  return EXIT_SUCCESS;
}

}  // namespace upkie_locomotion::spines::mock

int main(int argc, char** argv) {
  upkie_locomotion::spines::mock::CommandLineArguments args(
      {argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  }
  return upkie_locomotion::spines::mock::main(args);
}
