// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include "upkie/cpp/spine/AgentInterface.h"

namespace upkie::cpp::spine {

//! When sending stop cycles, send at least that many.
constexpr unsigned kNbStopCycles = 5;

//! States of the state machine.
enum class State : uint32_t {
  //! Send stop commands to all actuators
  kSendStops = 0,

  //! Reset all configured modules
  kReset = 1,

  //! Do nothing
  kIdle = 2,

  //! Write an observation to shared memory
  kObserve = 3,

  //! Read a new action from shared memory
  kAct = 4,

  //! Shut down the spine
  kShutdown = 5,

  //! Final termination
  kOver = 6
};

//! Events that may trigger transitions between states.
enum class Event : uint32_t {
  //! Beginning of a spine cycle
  kCycleBeginning = 0,

  //! End of a spine cycle
  kCycleEnd = 1,

  //! Caught an interruption signal
  kInterrupt = 2,
};

/*! Name of a state.
 *
 * \param[in] state State to name.
 */
constexpr const char* state_name(const State& state) noexcept {
  switch (state) {
    case State::kSendStops:
      return "State::kSendStops";
    case State::kReset:
      return "State::kReset";
    case State::kIdle:
      return "State::kIdle";
    case State::kObserve:
      return "State::kObserve";
    case State::kAct:
      return "State::kAct";
    case State::kShutdown:
      return "State::kShutdown";
    case State::kOver:
      return "State::kOver";
    default:
      break;
  }
  return "?";
}

/*! Spine state machine.
 *
 * The overall specification of the state machine is summarized in the
 * following diagram:
 *
 * \image html state-machine.png
 * \image latex state-machine.eps
 *
 * States have the following purposes:
 *
 * - Stop: do nothing, send stop commands to servos
 * - Reset: apply runtime configuration to actuation interface and observers
 *   - The reset state is not time-critical, *i.e.*, configuration can take
 *   time.
 * - Idle: do nothing
 * - Observe: write observation from the actuation interface
 * - Act: send action to the actuation interface
 * - Shutdown: terminal state, exit the control loop
 *
 * There are three possible events:
 *
 * - `begin`: beginning of a control cycle,
 * - `end`: end of a control cycle.
 * - `SIGINT`: the process received an interrupt signal.
 *
 * Guards, indicated between brackets, may involve two variables:
 *
 * - `req`: the current request from the agent.
 * - `stop_cycles`: the number of stop commands cycled in the current state
 *   (only available in "stop" and "shutdown" states).
 */
class StateMachine {
 public:
  /*! Initialize state machine.
   *
   * \param[in] interface Interface to communicate with the agent.
   */
  explicit StateMachine(AgentInterface& interface) noexcept;

  /*! Process a new event.
   *
   * \param[in] event New event.
   */
  void process_event(const Event& event) noexcept;

  //! Get current state.
  const State& state() const noexcept { return state_; }

  //! Whether we transition to the terminal state at the next end-cycle event.
  bool is_over_after_this_cycle() const noexcept {
    return (state_ == State::kShutdown && stop_cycles_ + 1u == kNbStopCycles);
  }

 private:
  /*! Go to a target state, triggering entry instructions if applicable.
   *
   * \param[in] next_state Target state.
   */
  void enter_state(const State& next_state) noexcept;

  //! Process Event::kCycleBeginning
  void process_cycle_beginning();

  //! Process Event::kCycleEnd
  void process_cycle_end();

 private:
  //! Shared-memory interface to communicate with the agent.
  AgentInterface& interface_;

  //! Current state
  State state_;

  //! Cycle counter for startup and shutdown phases
  unsigned stop_cycles_;
};

}  // namespace upkie::cpp::spine
