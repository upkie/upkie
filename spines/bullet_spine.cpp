/*
 * Copyright 2022 Stéphane Caron
 * Copyright 2023 Inria
 * SPDX-License-Identifier: Apache-2.0
 */

#include <vulp/actuation/BulletInterface.h>
#include <vulp/observation/ObserverPipeline.h>
#include <vulp/observation/sources/CpuTemperature.h>

#ifndef __APPLE__
#include <vulp/observation/sources/Joystick.h>
#endif

#include <vulp/spine/Spine.h>

#include <algorithm>
#include <cstdlib>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "upkie/config/layout.h"
#include "upkie/observers/FloorContact.h"
#include "upkie/observers/WheelOdometry.h"
#include "upkie/utils/datetime_now_string.h"
#include "upkie/version.h"

namespace spines::bullet {

using palimpsest::Dictionary;
using upkie::observers::FloorContact;
using upkie::observers::WheelOdometry;
using vulp::actuation::BulletInterface;
using vulp::observation::ObserverPipeline;
using vulp::observation::sources::CpuTemperature;

#ifndef __APPLE__
using vulp::observation::sources::Joystick;
#endif

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
      } else if (arg == "-v" || arg == "--version") {
        version = true;
      } else if (arg == "--log-dir") {
        log_dir = args.at(++i);
        spdlog::info("Command line: log_dir = {}", log_dir);
      } else if (arg == "--nb-substeps") {
        nb_substeps = std::stol(args.at(++i));
        spdlog::info("Command line: nb_substeps = {}", nb_substeps);
      } else if (arg == "--shm-name") {
        shm_name = args.at(++i);
        spdlog::info("Command line: shm_name = {}", shm_name);
      } else if (arg == "--show") {
        show = true;
      } else if (arg == "--space") {
        space = true;
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
    std::cout << "--log-dir <path>\n"
              << "    Path to a directory for output logs.\n";
    std::cout << "--nb-substeps <k>\n"
              << "    Number of simulation steps per action. "
              << "Makes spine pausing.\n";
    std::cout << "--shm-name <name>\n"
              << "    Name for IPC shared memory file.\n";
    std::cout << "--show\n"
              << "    Show the Bullet GUI.\n";
    std::cout << "--space\n"
              << "    No ground, no gravity, fly like an eagle!\n";
    std::cout << "--spine-frequency <frequency>\n"
              << "    Spine frequency in Hertz (default: 1000 Hz).\n";
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

  //! Number of simulation substeps
  unsigned nb_substeps = 0u;

  //! Name for the shared memory file
  std::string shm_name = "/vulp";

  //! Show Bullet GUI
  bool show = false;

  //! Space mode (no gravity)
  bool space = false;

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;

  //! Version flag
  bool version = false;
};

int main(const char* argv0, const CommandLineArguments& args) {
  ObserverPipeline observation;

  // Observation: CPU temperature
  auto cpu_temperature = std::make_shared<CpuTemperature>();
  observation.connect_source(cpu_temperature);

#ifndef __APPLE__
  // Observation: Joystick
  auto joystick = std::make_shared<Joystick>();
  if (joystick->present()) {
    spdlog::info("Joystick found");
    observation.connect_source(joystick);
  }
#endif

  // Observation: Floor contact
  FloorContact::Parameters floor_contact_params;
  floor_contact_params.dt = 1.0 / args.spine_frequency;
  floor_contact_params.upper_leg_joints = upkie::config::upper_leg_joints();
  floor_contact_params.wheels = upkie::config::wheel_joints();
  auto floor_contact = std::make_shared<FloorContact>(floor_contact_params);
  observation.append_observer(floor_contact);

  // Observation: Wheel odometry
  WheelOdometry::Parameters odometry_params;
  odometry_params.dt = 1.0 / args.spine_frequency;
  auto odometry = std::make_shared<WheelOdometry>(odometry_params);
  observation.append_observer(odometry);

  // Note that we don't lock memory in this spine. Otherwise Bullet will yield
  // a "b3AlignedObjectArray reserve out-of-memory" error below.

  // Simulator
  const auto servo_layout = upkie::config::servo_layout();
  const double base_altitude = args.space ? 0.0 : 0.6;  // [m]
  BulletInterface::Parameters bullet_params(Dictionary{});
  bullet_params.argv0 = argv0;
  bullet_params.dt = 1.0 / args.spine_frequency;
  bullet_params.floor = !args.space;
  bullet_params.gravity = !args.space;
  bullet_params.gui = args.show;
  bullet_params.position_base_in_world = Eigen::Vector3d(0., 0., base_altitude);
  bullet_params.urdf_path = "external/upkie_description/urdf/upkie.urdf";
  BulletInterface interface(servo_layout, bullet_params);

  // Spine
  Spine::Parameters spine_params;
  spine_params.frequency = args.spine_frequency;
  const auto now = upkie::utils::datetime_now_string();
  spine_params.log_path = args.log_dir + "/" + now + "_bullet_spine.mpack";
  spine_params.shm_name = args.shm_name;
  spdlog::info("Spine data logged to {}", spine_params.log_path);
  Spine spine(spine_params, interface, observation);
  if (args.nb_substeps == 0u) {
    spine.run();
  } else /* args.nb_substeps > 0 */ {
    spdlog::set_level(spdlog::level::warn);
    spine.simulate(args.nb_substeps);
  }

  return EXIT_SUCCESS;
}

}  // namespace spines::bullet

int main(int argc, char** argv) {
  spines::bullet::CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  } else if (args.version) {
    std::cout << "Upkie bullet spine " << upkie::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return spines::bullet::main(argv[0], args);
}
