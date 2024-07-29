// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/spine/StateMachine.h"

#include <spdlog/spdlog.h>

#include "upkie/cpp/utils/handle_interrupts.h"

namespace upkie::cpp::spine {

StateMachine::StateMachine(AgentInterface& interface) noexcept
    : interface_(interface), state_(State::kSendStops), stop_cycles_(0u) {
  enter_state(State::kSendStops);  // sets request to Request::kNone
}

void StateMachine::process_event(const Event& event) noexcept {
  switch (event) {
    case Event::kInterrupt:
      if (state_ != State::kShutdown) {
        enter_state(State::kShutdown);
      }
      break;
    case Event::kCycleBeginning:
      process_cycle_beginning();
      break;
    case Event::kCycleEnd:
      process_cycle_end();
      break;
    default:
      spdlog::error("Invalid event '{}', shutting down...", event);
      enter_state(State::kShutdown);
      break;
  }
}

void StateMachine::process_cycle_beginning() {
  const Request request = interface_.request();
  switch (state_) {
    case State::kIdle:
      switch (request) {
        case Request::kNone:
          break;
        case Request::kObservation:
          enter_state(State::kObserve);
          break;
        case Request::kAction:
          enter_state(State::kAct);
          break;
        case Request::kStart:
          spdlog::warn(
              "Request::kStart is invalid from State::kIdle! Stop the spine "
              "then start it again");
          enter_state(State::kIdle);  // reset request
          break;
        case Request::kStop:
          spdlog::info("Stop requested by agent");
          enter_state(State::kSendStops);
          break;
        default:
          interface_.set_request(Request::kError);
          break;
      }
      break;
    case State::kReset:
      spdlog::warn(
          "Event::kCycleBeginning should not happen from State::kReset");
      break;
    case State::kObserve:
      spdlog::warn(
          "Event::kCycleBeginning should not happen from State::kObserve");
      break;
    case State::kAct:
      spdlog::warn("Event::kCycleBeginning should not happen from State::kAct");
      break;
    case State::kSendStops:
      switch (request) {
        case Request::kNone:
          break;
        case Request::kObservation:
        case Request::kAction:
          interface_.set_request(Request::kError);
          break;
        case Request::kStart:
          if (stop_cycles_ >= kNbStopCycles) {
            enter_state(State::kReset);
          }
          break;
        case Request::kStop:
          enter_state(State::kSendStops);
          break;
        default:
          interface_.set_request(Request::kError);
          break;
      }
      break;
    case State::kShutdown:
    case State::kOver:
      break;
    default:
      spdlog::error("Invalid FSM state '{}', shutting down...", state_);
      enter_state(State::kShutdown);
      break;
  }
}

void StateMachine::process_cycle_end() {
  switch (state_) {
    case State::kIdle:
      break;
    case State::kReset:
      spdlog::info("Start requested by agent");
      enter_state(State::kIdle);
      break;
    case State::kSendStops:
      if (++stop_cycles_ <= kNbStopCycles) {
        spdlog::info("Stop cycle {} / {}", stop_cycles_, kNbStopCycles);
      }
      break;
    case State::kObserve:
      enter_state(State::kIdle);
      break;
    case State::kAct:
      enter_state(State::kIdle);
      break;
    case State::kShutdown:
      if (++stop_cycles_ == kNbStopCycles) {
        enter_state(State::kOver);
      }
      spdlog::info("Shutdown cycle {} / {}", stop_cycles_, kNbStopCycles);
      break;
    case State::kOver:
      break;
    default:
      spdlog::error("Invalid FSM state '{}', shutting down...", state_);
      enter_state(State::kShutdown);
      break;
  }
}

void StateMachine::enter_state(const State& next_state) noexcept {
  switch (next_state) {
    case State::kIdle:
      interface_.set_request(Request::kNone);
      break;
    case State::kReset:
    case State::kObserve:
    case State::kAct:
      break;
    case State::kSendStops:
    case State::kShutdown:
      interface_.set_request(Request::kNone);
      stop_cycles_ = 0u;
      break;
    case State::kOver:
      break;
    default:
      spdlog::error("Cannot go to state '{}', shutting down...",
                    state_name(state_));
      return enter_state(State::kShutdown);
  }
  state_ = next_state;
}

}  // namespace upkie::cpp::spine
