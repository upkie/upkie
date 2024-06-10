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

#include "upkie/utils/datetime_now_string.h"

namespace upkie::utils {

/*! Get path to a fresh log file.
 *
 * \param[in] spine_name Name of the spine.
 * \param[in] log_dir Path to the logging directory.
 * \return Path to a fresh log file.
 *
 * Uses the same format as the Python utility function.
 */
inline const std::string get_log_path(const std::string& spine_name,
                                      const std::string& log_dir) {
  const auto now = datetime_now_string();
  const std::string prefix = log_dir + "/" + now + "_" + spine_name;

  char hostname[512];
  int return_code = gethostname(hostname, 512);
  if (return_code != 0) {
    spdlog::warn("Could not get the robot's hostname, errno = {}", errno);
    return prefix + ".mpack";
  }
  return prefix + "_" + hostname + ".mpack";
}

}  // namespace upkie::utils
