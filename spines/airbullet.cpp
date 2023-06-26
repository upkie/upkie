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

#include <vulp/actuation/BulletInterface.h>
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

#include "upkie/spines/upkie_layout.h"
#include "upkie/utils/datetime_now_string.h"

namespace upkie::spines::airbullet {

using palimpsest::Dictionary;
using vulp::actuation::BulletInterface;
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
    std::cout << "--spine-frequency <frequency>\n"
              << "    Spine frequency in Hertz (default: 1000 Hz).\n";
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

  //! Spine frequency in Hz.
  unsigned spine_frequency = 1000u;
};

int main(const char* argv0, const CommandLineArguments& args) {
  const auto servo_layout = upkie_layout::servo_layout();
  BulletInterface::Parameters bullet_params(Dictionary{});
  bullet_params.argv0 = argv0;
  bullet_params.dt = 1.0 / args.spine_frequency;
  bullet_params.floor = false;
  bullet_params.gravity = false;
  bullet_params.gui = args.show;
  bullet_params.position_init_base_in_world = Eigen::Vector3d(0., 0., 0.);
  bullet_params.urdf_path = "external/upkie_description/urdf/upkie.urdf";
  BulletInterface interface(servo_layout, bullet_params);

  Spine::Parameters spine_params;
  spine_params.frequency = args.spine_frequency;
  const auto now = upkie::utils::datetime_now_string();
  spine_params.log_path = args.log_dir + "/" + now + "_airbullet_spine.mpack";
  spine_params.shm_name = args.shm_name;
  spdlog::info("Spine data logged to {}", spine_params.log_path);
  vulp::observation::ObserverPipeline observation;
  Spine spine(spine_params, interface, observation);
  if (args.nb_substeps == 0u) {
    spine.run();
  } else /* args.nb_substeps > 0 */ {
    spdlog::set_level(spdlog::level::warn);
    spine.simulate(args.nb_substeps);
  }

  return EXIT_SUCCESS;
}

}  // namespace upkie::spines::airbullet

int main(int argc, char** argv) {
  upkie::spines::airbullet::CommandLineArguments args({argv + 1, argv + argc});
  if (args.error) {
    return EXIT_FAILURE;
  } else if (args.help) {
    args.print_usage(argv[0]);
    return EXIT_SUCCESS;
  }
  return upkie::spines::airbullet::main(argv[0], args);
}
