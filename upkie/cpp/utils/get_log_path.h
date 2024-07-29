// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2024 Inria

#pragma once

#include <spdlog/spdlog.h>
#include <unistd.h>

#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include "upkie/cpp/utils/datetime_now_string.h"

namespace upkie::cpp::utils {

/*! Get path to a fresh log file.
 *
 * \param[in] log_dir Path to the logging directory.
 * \param[in] spine_name Name of the spine.
 * \return Path to a fresh log file.
 *
 * Uses the same format as the Python utility function.
 */
inline const std::string get_log_path(const std::string& log_dir,
                                      const std::string& spine_name) {
  const std::string prefix = log_dir + "/" + datetime_now_string() + "_";
  char hostname[512];
  int return_code = gethostname(hostname, 512);
  if (return_code != 0) {
    spdlog::warn("Could not get the robot's hostname, errno = {}", errno);
    return prefix + spine_name + ".mpack";
  }
  return prefix + hostname + "_" + spine_name + ".mpack";
}

}  // namespace upkie::cpp::utils
