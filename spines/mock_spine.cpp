// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023 Inria

#include <algorithm>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "spines/common/controllers.h"
#include "spines/common/observers.h"
#include "spines/common/sensors.h"
#include "upkie/cpp/controllers/ControllerPipeline.h"
#include "upkie/cpp/interfaces/MockInterface.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/sensors/SensorPipeline.h"
#include "upkie/cpp/spine/Spine.h"
#include "upkie/cpp/utils/get_log_path.h"
#include "upkie/cpp/utils/realtime.h"
#include "upkie/cpp/version.h"

using palimpsest::Dictionary;
using spines::common::make_controllers;
using spines::common::make_observers;
using spines::common::make_sensors;
using upkie::cpp::controllers::ControllerPipeline;
using upkie::cpp::interfaces::MockInterface;
using upkie::cpp::observers::ObserverPipeline;
using upkie::cpp::sensors::SensorPipeline;
using upkie::cpp::spine::Spine;

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
      } else if (arg == "--pipeline") {
        pipeline = args.at(++i);
        spdlog::info("Command line: pipeline = {}", pipeline);
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
    std::cout << "--pipeline <name>\n"
              << "    Pipeline name (e.g., 'wheel_balancer').\n";
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

  //! Pipeline name
  std::string pipeline = "servo";

  //! CPUID for the spine thread (-1 to disable realtime).
  int spine_cpu = -1;

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;

  //! Version flag
  bool version = false;
};

//! Build and run the mock spine.
int run_spine(const CommandLineArguments& args) {
  if (!upkie::cpp::utils::lock_memory()) {
    spdlog::error("could not lock process memory to RAM");
    return -4;
  }

  SensorPipeline sensors = make_sensors(/* joystick_required = */ false);
  ObserverPipeline observers = make_observers(args.spine_frequency);
  ControllerPipeline controllers =
      make_controllers(args.pipeline, args.spine_frequency);

  // Mock actuators
  const double dt = 1.0 / args.spine_frequency;
  MockInterface actuation(dt);

  // Spine
  Spine::Parameters spine_params;
  spine_params.cpu = args.spine_cpu;
  spine_params.frequency = args.spine_frequency;
  spine_params.log_path =
      upkie::cpp::utils::get_log_path(args.log_dir, "mock_spine");
  spdlog::info("Spine data logged to {}", spine_params.log_path);
  Spine spine(spine_params, actuation, sensors, observers, controllers);
  spine.run();

  return EXIT_SUCCESS;
}

int main(int argc, char** argv) {
  CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  } else if (args.version) {
    std::cout << "Upkie mock spine " << upkie::cpp::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return run_spine(args);
}
