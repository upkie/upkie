// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <fcntl.h>
#include <spdlog/spdlog.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <cstring>
#include <string>

#include "upkie/cpp/spine/Request.h"

/*! Inter-process communication protocol with the spine.
 *
 * Design notes: one alternative was a proper FSM with a clear distinction
 * between states (what the spine is doing) and transitions (agent requests,
 * spine replies, etc.). This could have allowed more granularity, such as the
 * spine doing nothing (instead of sending stops) when it is idle, however at
 * the cost of more complexity. We choose to go for the simpler (riskier)
 * version where both agent and spine directly manipulate the agenda.
 */
namespace upkie::spine {

/*! Memory map to shared memory.
 *
 * On memory mapping, shared memory and memory-mapped files:
 * https://w3.cs.jmu.edu/kirkpams/OpenCSF/Books/csf/html/MMap.html
 */
class AgentInterface {
 public:
  /*! Open interface to the agent at a given shared-memory file.
   *
   * \param[in] name Name of the shared memory file (e.g. "/upkie").
   * \param[in] size Size in bytes.
   *
   * Shared memory objects are available as files in ``/dev/shm``.
   */
  AgentInterface(const std::string& name, size_t size);

  //! Unmap memory and unlink shared memory object
  ~AgentInterface();

  /*! Set current request in shared memory.
   *
   * \param[in] request New request.
   */
  void set_request(Request request);

  /*! Write data to the data buffer in shared memory.
   *
   * \param[in] data Data to write.
   * \param[in] size Number of bytes to write.
   */
  void write(char* data, size_t size);

  //! Get current request from shared memory
  Request request() const { return static_cast<Request>(*mmap_request_); }

  //! Get size of current data buffer
  uint32_t size() const { return *mmap_size_; }

  //! Get pointer to data buffer
  const char* data() const { return mmap_data_; }

 private:
  //! Name of the shared memory object
  std::string name_;

  //! Size of the shared memory in bytes
  size_t size_;

  //! Address of the shared memory map
  void* mmap_;

  //! Pointer to the current request in shared memory
  uint32_t* mmap_request_;

  //! Pointer to the data size frame in shared memory
  uint32_t* mmap_size_;

  //! Pointer to the data string in shared memory
  char* mmap_data_;
};

}  // namespace upkie::spine
