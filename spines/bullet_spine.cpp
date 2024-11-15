// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023 Inria

#include <algorithm>
#include <cstdlib>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "upkie/cpp/actuation/BulletInterface.h"
#include "upkie/cpp/observers/BaseOrientation.h"
#include "upkie/cpp/observers/FloorContact.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/observers/WheelOdometry.h"
#include "upkie/cpp/sensors/CpuTemperature.h"
#include "upkie/cpp/spine/Spine.h"
#include "upkie/cpp/utils/get_log_path.h"
#include "upkie/cpp/version.h"

#ifndef __APPLE__
#include "upkie/cpp/sensors/Joystick.h"
#endif

namespace spines::bullet {

using palimpsest::Dictionary;
using upkie::cpp::actuation::BulletInterface;
using upkie::cpp::observers::BaseOrientation;
using upkie::cpp::observers::FloorContact;
using upkie::cpp::observers::ObserverPipeline;
using upkie::cpp::observers::WheelOdometry;
using upkie::cpp::sensors::CpuTemperature;
using upkie::cpp::spine::Spine;

#ifndef __APPLE__
using upkie::cpp::sensors::Joystick;
#endif

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
      } else if (arg == "--extra-urdf-path") {
        extra_urdf_paths.push_back(args.at(++i));
        spdlog::info("Command line: extra-urdf-path = {}",
                     extra_urdf_paths.back());
      } else if (arg == "--spine-frequency") {
        spine_frequency = std::stol(args.at(++i));
        spdlog::info("Command line: spine_frequency = {} Hz", spine_frequency);
      } else if (arg == "--robot-variant") {
        robot_variant = args.at(++i);
        spdlog::info("Command line: robot_variant = {}", robot_variant);
      } else if (arg == "--mass-randomization-epsilon") {
        mass_randomization_epsilon = std::stod(args.at(++i));
        spdlog::info("Command line: mass_randomization_epsilon = {} [kg]",
                     mass_randomization_epsilon);
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
    std::cout << "--extra-urdf-path\n"
              << "    Load extra URDFs into the environment.\n";
    std::cout << "--spine-frequency <frequency>\n"
              << "    Spine frequency in Hertz (default: 1000 Hz).\n";
    std::cout << "--robot-variant <variant>\n"
              << "    Robot variant (default: '').\n";
    std::cout << "--mass-randomization-epsilon <epsilon>\n"
              << "    Randomize masses of the robot links.\n";
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
  std::string shm_name = "/upkie";

  //! Show Bullet GUI
  bool show = false;

  //! Space mode (no ground, no gravity)
  bool space = false;

  //! Extra URDF paths
  std::vector<std::string> extra_urdf_paths;

  //! Robot variant
  std::string robot_variant;

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;

  //! Mass randomization epsilon
  double mass_randomization_epsilon = 0.0;

  //! Version flag
  bool version = false;
};

int clear_shared_memory(const std::string& name) {
  const char* shm_name = name.c_str();
  int file_descriptor = ::shm_open(shm_name, O_RDWR, 0666);
  if (file_descriptor < 0) {
    if (errno == ENOENT) {
      return EXIT_SUCCESS;
    } else if (errno == EINVAL) {
      spdlog::error("Cannot clear shared memory (EINVAL: name '{}' invalid)",
                    shm_name);
      return EXIT_FAILURE;
    } else {
      spdlog::error(
          "Cannot clear shared memory (error opening '{}', error number: {})",
          shm_name, errno);
      return EXIT_FAILURE;
    }
  }
  if (::shm_unlink(shm_name) < 0) {
    spdlog::error(
        "Failed to unlink shared memory (name: '{}', error number: {})",
        shm_name, errno);
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int main(const char* argv0, const CommandLineArguments& args) {
  ObserverPipeline observation;

  // Clear any existing shared-memory file
  if (clear_shared_memory(args.shm_name) != EXIT_SUCCESS) {
    return EXIT_FAILURE;
  }

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
  const double base_altitude = args.space ? 0.0 : 0.6;  // [m]
  BulletInterface::Parameters bullet_params(Dictionary{});
  bullet_params.argv0 = argv0;
  bullet_params.dt = 1.0 / args.spine_frequency;
  bullet_params.floor = !args.space;
  bullet_params.gravity = !args.space;
  bullet_params.gui = args.show;
  bullet_params.mass_randomization_epsilon = args.mass_randomization_epsilon;
  bullet_params.position_base_in_world = Eigen::Vector3d(0., 0., base_altitude);
  if (args.robot_variant.empty()) {
    bullet_params.robot_urdf_path =
        "external/upkie_description/urdf/upkie.urdf";
  } else {
    bullet_params.robot_urdf_path =
        "external/upkie_description/urdf/upkie_" + args.robot_variant + ".urdf";
  }
  bullet_params.env_urdf_paths = args.extra_urdf_paths;
  BulletInterface interface(bullet_params);

  // Spine
  Spine::Parameters spine_params;
  spine_params.frequency = args.spine_frequency;
  spine_params.log_path =
      upkie::cpp::utils::get_log_path(args.log_dir, "bullet_spine");
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
    std::cout << "Upkie bullet spine " << upkie::cpp::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return spines::bullet::main(argv[0], args);
}
