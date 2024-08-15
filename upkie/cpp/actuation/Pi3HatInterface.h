// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     SPDX-License-Identifier: Apache-2.0
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 */

#pragma once

#include <mjbots/pi3hat/pi3hat.h>
#include <pthread.h>
#include <spdlog/spdlog.h>

#include <Eigen/Geometry>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "upkie/cpp/actuation/ImuData.h"
#include "upkie/cpp/actuation/Interface.h"
#include "upkie/cpp/actuation/moteus/protocol.h"
#include "upkie/cpp/utils/realtime.h"

namespace upkie::cpp::actuation {

/*! Interface to moteus controllers.
 *
 * Internally it uses a background thread to operate the pi3hat, enabling the
 * main thread to perform work while servo communication is taking place.
 */
class Pi3HatInterface : public Interface {
 public:
  /*! Configure interface and spawn CAN thread.
   *
   * \param[in] layout Servo layout.
   * \param[in] can_cpu CPUID of the core to run the CAN thread on.
   * \param[in] pi3hat_config Configuration for the pi3hat.
   */
  Pi3HatInterface(const ServoLayout& layout, const int can_cpu,
                  const ::mjbots::pi3hat::Pi3Hat::Configuration& pi3hat_config);

  //! Stop CAN thread
  ~Pi3HatInterface();

  /*! Reset interface.
   *
   * \param[in] config Additional configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Write actuation-interface observations to dictionary.
   *
   * \param[out] observation Dictionary to write to.
   */
  void observe(Dictionary& observation) const override;

  /*! Process a new action dictionary.
   *
   * \param[in] action Action to read commands from.
   */
  void process_action(const Dictionary& action) override;

  /*! Spin a new communication cycle.
   *
   * \param[in] callback Function to call when the cycle is over.
   *
   * The callback will be invoked from an arbitrary thread when the
   * communication cycle has completed. All memory pointed to by \p data must
   * remain valid until the callback is invoked.
   */
  void cycle(std::function<void(const moteus::Output&)> callback) final;

 private:
  /*! Main loop of the CAN thread.
   *
   * Synchronizes with \ref cycle via the internal condition variable.
   */
  void run_can_thread();

  /*! Execute one communication cycle on the CAN bus.
   *
   * Also, request the latest filtered attitude from the pi3hat.
   */
  moteus::Output cycle_can_thread();

 private:
  //! CPUID of the core to run the CAN thread on.
  const int can_cpu_;

  // pi3hat configuration
  const ::mjbots::pi3hat::Pi3Hat::Configuration pi3hat_config_;

  //! Mutex associated with \ref can_wait_condition_
  std::mutex mutex_;

  //! Condition variable to notify the CAN thread to spin a new cycle.
  std::condition_variable can_wait_condition_;

  //! True if and only if a CAN communication cycle is under way.
  bool ongoing_can_cycle_ = false;

  //! CAN thread exits when it is notified and this boolean is true.
  bool done_ = false;

  //! Callback function called upon completion of a CAN cycle
  std::function<void(const moteus::Output&)> callback_;

  //! Thread for CAN communication cycles
  std::thread can_thread_;

  //! Internal Pi3Hat interface. Only use from the CAN thread.
  std::unique_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;

  /*! TX CAN frames
   *
   * These are kept persistently so that no memory allocation is required in
   * steady state.
   */
  std::vector<mjbots::pi3hat::CanFrame> tx_can_;

  /*! RX CAN frames
   *
   * These are kept persistently so that no memory allocation is required in
   * steady state.
   */
  std::vector<mjbots::pi3hat::CanFrame> rx_can_;

  //! Latest attitude read from the pi3hat
  ::mjbots::pi3hat::Attitude attitude_;
};

}  // namespace upkie::cpp::actuation
