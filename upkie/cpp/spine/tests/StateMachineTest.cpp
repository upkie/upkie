// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "upkie/cpp/spine/AgentInterface.h"
#include "upkie/cpp/spine/StateMachine.h"
#include "upkie/cpp/utils/random_string.h"

namespace upkie::spine {

class StateMachineTest : public ::testing::Test {
 protected:
  //! Prepare state machine for a new test
  void SetUp() override {
    const size_t shm_size = 1 * (1 << 20);
    const std::string shm_name = std::string("/") + random_string();
    agent_interface_ = std::make_unique<AgentInterface>(shm_name, shm_size);
    state_machine_ = std::make_unique<StateMachine>(*agent_interface_);
  }

  //! Get current request
  Request request() { return agent_interface_->request(); }

  //! Get current state
  State state() { return state_machine_->state(); }

  //! Convenience function to set the request directly
  void set_request(const Request& request) {
    agent_interface_->set_request(request);
  }

  //! Test actuator interface
  std::unique_ptr<AgentInterface> agent_interface_;

  //! Test state machine
  std::unique_ptr<StateMachine> state_machine_;
};

TEST_F(StateMachineTest, StartsInStopState) {
  ASSERT_EQ(state(), State::kSendStops);
  ASSERT_EQ(request(), Request::kNone);
}

TEST_F(StateMachineTest, RestopsWork) {
  ASSERT_EQ(state(), State::kSendStops);

  // Requesting a stop from stop state is idempotent
  set_request(Request::kStop);
  state_machine_->process_event(Event::kCycleBeginning);
  ASSERT_EQ(request(), Request::kNone);
  ASSERT_EQ(state(), State::kSendStops);
  state_machine_->process_event(Event::kCycleEnd);
  ASSERT_EQ(request(), Request::kNone);
  ASSERT_EQ(state(), State::kSendStops);
}

TEST_F(StateMachineTest, Startup) {
  ASSERT_EQ(state(), State::kSendStops);

  set_request(Request::kStart);

  // Wait for the required number of stop cycles
  for (unsigned cycle = 0; cycle < kNbStopCycles; ++cycle) {
    state_machine_->process_event(Event::kCycleBeginning);
    ASSERT_EQ(state(), State::kSendStops);
    state_machine_->process_event(Event::kCycleEnd);
    ASSERT_EQ(state(), State::kSendStops);
  }

  // Regular startup sequence
  state_machine_->process_event(Event::kCycleBeginning);
  ASSERT_EQ(state(), State::kReset);
  state_machine_->process_event(Event::kCycleEnd);
  ASSERT_EQ(state(), State::kIdle);
  ASSERT_EQ(request(), Request::kNone);

  // Requesting a second start has no effect
  set_request(Request::kStart);
  state_machine_->process_event(Event::kCycleBeginning);
  ASSERT_EQ(request(), Request::kNone);
  ASSERT_EQ(state(), State::kIdle);
  state_machine_->process_event(Event::kCycleEnd);
  ASSERT_EQ(state(), State::kIdle);
  ASSERT_EQ(request(), Request::kNone);
}

TEST_F(StateMachineTest, ObserveWhenSendingStopsFails) {
  ASSERT_EQ(state(), State::kSendStops);

  set_request(Request::kObservation);
  state_machine_->process_event(Event::kCycleBeginning);
  ASSERT_EQ(state(), State::kSendStops);
  ASSERT_EQ(request(), Request::kError);
}

}  // namespace upkie::spine
