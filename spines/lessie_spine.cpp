#include <unistd.h>
#include "vulp/actuation/LessieInterface.h"
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

#include "upkie/config/layout.h"
#include "upkie/observers/FloorContact.h"
#include "upkie/observers/WheelOdometry.h"
#include "upkie/utils/datetime_now_string.h"
#include "upkie/version.h"
#include "configs.h"

namespace spines::lessie {

// using Pi3Hat = ::mjbots::pi3hat::Pi3Hat;
using palimpsest::Dictionary;
using upkie::observers::FloorContact;
using upkie::observers::WheelOdometry;
using vulp::actuation::LessieInterface;
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
  unsigned spine_frequency = 1002u;

  //! Error flag.
  bool version = false;
};

inline bool calibration_needed() {
  std::ifstream file{"/tmp/rezero_success"};
  const bool file_found = file.is_open();
  return !file_found;
}

inline const std::string get_log_path(const std::string& log_dir) {
  const auto now = upkie::utils::datetime_now_string();
  const std::string prefix = log_dir + "/" + now + "_pi3hat_spine";

  char hostname[512];
  int return_code = gethostname(hostname, 512);
  if (return_code != 0) {
    spdlog::error("Could not get the robot's hostname, errno = {}", errno);
    return prefix + ".mpack";
  }
  return prefix + "_" + hostname + ".mpack";
}

int main(const CommandLineArguments& args) {
  // if (calibration_needed()) {
  //   spdlog::error("Calibration needed: did you run `upkie_tool rezero`?");
  //   return -3;
  // }
  if (!vulp::utils::lock_memory()) {
    spdlog::error("Could not lock process memory to RAM");
    return -4;
  }

  ObserverPipeline observation;

  // Observation: CPU temperature
  auto cpu_temperature = std::make_shared<CpuTemperature>();
  observation.connect_source(cpu_temperature);

  spdlog::info("spine_frequency:{}", to_string(args.spine_frequency));

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
  observation.connect_source(joystick);

  // Observation: Wheel odometry
  WheelOdometry::Parameters odometry_params;
  odometry_params.dt = 1.0 / args.spine_frequency;
  auto odometry = std::make_shared<WheelOdometry>(odometry_params);
  observation.append_observer(odometry);

  try {
    // lessie configuration
    vulp::actuation::WiringpiConfig config = ::LESSIE_MARK1();
    LessieInterface interface(config.servoLayout(), args.can_cpu, config, false);

    // Spine
    Spine::Parameters spine_params;
    spine_params.cpu = args.spine_cpu;
    spine_params.frequency = args.spine_frequency;
    spine_params.log_path = get_log_path(args.log_dir);
    spdlog::info("Spine data logged to {}", spine_params.log_path);
    Spine spine(spine_params, interface, observation);
    spine.run();
  } catch (const std::exception& error) {
    std::string message = error.what();
    spdlog::error(message);
    if (message.find("/dev/mem") != std::string::npos) {
      spdlog::info("did you run with sudo?");
    }
    return -5;
  }

  return EXIT_SUCCESS;
}

}  // namespace spines::lessie

int main(int argc, char** argv) {
  spines::lessie::CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  } else if (args.version) {
    std::cout << "lessie spine " << upkie::kVersion << "\n";
    return EXIT_SUCCESS;
  }
  return spines::lessie::main(args);
}
