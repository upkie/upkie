// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>

namespace upkie::cpp::utils {

/*! Clear an existing shared-memory file.
 *
 * \param[in] name Name of the shared-memory file to remove if it exists.
 * \return EXIT_SUCCESS if the file is cleared, EXIT_FAILURE otherwise.
 */
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

}  // namespace upkie::cpp::utils
