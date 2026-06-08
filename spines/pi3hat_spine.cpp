// SPDX-License-Identifier: Apache-2.0

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

#include "spines/common/controllers.h"
#include "spines/common/observers.h"
#include "spines/common/sensors.h"
#include "upkie/cpp/controllers/ControllerPipeline.h"
#include "upkie/cpp/exceptions/UpkieError.h"
#include "upkie/cpp/interfaces/MockInterface.h"
#include "upkie/cpp/interfaces/Pi3HatInterface.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/sensors/CpuTemperature.h"
#include "upkie/cpp/sensors/Joystick.h"
#include "upkie/cpp/sensors/SensorPipeline.h"
#include "upkie/cpp/spine/Spine.h"
#include "upkie/cpp/utils/get_log_path.h"
#include "upkie/cpp/utils/realtime.h"
#include "upkie/cpp/version.h"

using Pi3Hat = ::mjbots::pi3hat::Pi3Hat;
using palimpsest::Dictionary;
using spines::common::make_controllers;
using spines::common::make_observers;
using spines::common::make_sensors;
using upkie::cpp::controllers::ControllerPipeline;
using upkie::cpp::interfaces::MockInterface;
using upkie::cpp::interfaces::Pi3HatInterface;
using upkie::cpp::observers::ObserverPipeline;
using upkie::cpp::sensors::CpuTemperature;
using upkie::cpp::sensors::Joystick;
using upkie::cpp::sensors::SensorPipeline;
using upkie::cpp::spine::Spine;

//! Command-line arguments for the pi3hat spine.
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
      } else if (arg == "--mock") {
        mock = true;
        spdlog::info("Command line: mock mode enabled");
      } else if (arg == "--readonly") {
        readonly = true;
        spdlog::info("Command line: readonly mode enabled");
      } else if (arg == "--pipeline") {
        pipeline = args.at(++i);
        spdlog::info("Command line: pipeline = {}", pipeline);
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
    if (mock && readonly) {
      spdlog::error("Cannot use both --mock and --readonly");
      error = true;
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
    std::cout << "--mock\n"
              << "    Run in mock mode (no actuator communication).\n";
    std::cout << "--readonly\n"
              << "    Run in readonly mode (observe state, don't command).\n";
    std::cout << "--pipeline <name>\n"
              << "    Pipeline name (e.g., 'wheel_balancer').\n";
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

  //! Mock mode flag (no actuator communication).
  bool mock = false;

  //! Readonly mode flag (observe state, don't command actuators).
  bool readonly = false;

  //! Log directory
  std::string log_dir = "";

  //! Pipeline name
  std::string pipeline = "servo";

  //! Name for the shared memory file.
  std::string shm_name = "/upkie";

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

/*! Ensure all CPU cores use the "performance" frequency governor.
 *
 * The pi3hat spine is latency-sensitive, so we want all cores running at
 * maximum frequency rather than dynamically scaling. On the Raspberry Pi,
 * all CPU governors are tied together, so setting CPU 0 applies to all.
 */
inline void configure_cpufreq() {
  const std::string path =
      "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor";
  std::ifstream reader(path);
  if (!reader.is_open()) {
    spdlog::warn("Could not open {} for reading", path);
    return;
  }
  std::string governor;
  std::getline(reader, governor);
  reader.close();
  if (governor == "performance") {
    return;
  }

  spdlog::info("Setting CPU frequency governor from \"{}\" to \"performance\"",
               governor);
  std::ofstream writer(path);
  if (!writer.is_open()) {
    spdlog::warn("Could not open {} for writing", path);
    return;
  }
  writer << "performance";
  writer.close();

  // Verify the write took effect
  std::ifstream verifier(path);
  std::string new_governor;
  std::getline(verifier, new_governor);
  verifier.close();
  if (new_governor != "performance") {
    spdlog::warn(
        "CPU frequency governor is \"{}\" after write, "
        "expected \"performance\"",
        new_governor);
  }
}

//! Build and run the pi3hat spine.
int run_spine(const CommandLineArguments& args) {
  if (!args.mock) {
    configure_cpufreq();
    if (calibration_needed()) {
      spdlog::error("Calibration needed: did you run `upkie_tool rezero`?");
      return -3;
    }
  }
  if (!upkie::cpp::utils::lock_memory()) {
    spdlog::error("Could not lock process memory to RAM");
    return -4;
  }

  const std::string spine_name = args.mock       ? "pi3hat_spine_mock"
                                 : args.readonly ? "pi3hat_spine_readonly"
                                                 : "pi3hat_spine";

  try {
    // Make pipelines
    const bool joystick_required = !args.mock && !args.readonly;
    SensorPipeline sensors = make_sensors(joystick_required);
    ObserverPipeline observers = make_observers(args.spine_frequency);
    ControllerPipeline controllers =
        make_controllers(args.pipeline, args.spine_frequency);

    // Spine parameters
    Spine::Parameters spine_params;
    spine_params.cpu = args.spine_cpu;
    spine_params.frequency = args.spine_frequency;
    spine_params.log_path =
        upkie::cpp::utils::get_log_path(args.log_dir, spine_name);
    spine_params.shm_name = args.shm_name;
    spine_params.readonly = args.readonly;
    spdlog::info("Spine data logged to {}", spine_params.log_path);

    if (args.mock) {
      // Mock interface
      const double dt = 1.0 / args.spine_frequency;
      MockInterface interface(dt);
      Spine spine(spine_params, interface, sensors, observers, controllers);
      spine.run();
    } else {
      // pi3hat configuration
      Pi3Hat::Configuration pi3hat_config;
      pi3hat_config.attitude_rate_hz = args.attitude_frequency;
      pi3hat_config.mounting_deg.pitch = 0.;
      pi3hat_config.mounting_deg.roll = 0.;
      pi3hat_config.mounting_deg.yaw = 0.;

      // pi3hat interface
      Pi3HatInterface interface(args.can_cpu, pi3hat_config);
      Spine spine(spine_params, interface, sensors, observers, controllers);
      spine.run();
    }
  } catch (const upkie::cpp::exceptions::UpkieError& error) {
    spdlog::error("Upkie error: {}", error.what());
    return -2;
  } catch (const ::mjbots::pi3hat::Error& error) {
    std::string message = error.what();
    spdlog::error("pi3hat error: {}", message);
    if (message.find("/dev/mem") != std::string::npos) {
      spdlog::info("did you run with sudo?");
    }
    return -5;
  }

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
    std::cout << "Upkie pi3hat spine " << upkie::cpp::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return run_spine(args);
}
