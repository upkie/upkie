// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/Interface.h"
#include "upkie/cpp/actuation/tests/coffee_machine_layout.h"

namespace upkie {

class SmallInterface : public Interface {
 public:
  using Interface::Interface;

  void reset(const Dictionary& config) override {}

  void cycle(std::function<void(const moteus::Output&)> callback) final {}

  void observe(Dictionary& observation) const override {}

  void process_action(const Dictionary& action) override {}
};

class InterfaceTest : public ::testing::Test {
 protected:
  //! Set up a new test fixture
  void SetUp() override {
    const auto layout = get_coffee_machine_layout();
    interface_ = std::make_unique<SmallInterface>(layout);
  }

 protected:
  //! Actuation interface
  std::unique_ptr<Interface> interface_;

  //! Servo commands placeholder
  std::vector<moteus::ServoCommand> commands_;

  //! Servo replies placeholder
  std::vector<moteus::ServoReply> replies_;

  //! Data buffer to call interface_->cycle()
  moteus::Data data_;
};

TEST_F(InterfaceTest, CommandsAreInitialized) {
  ASSERT_EQ(interface_->commands().size(), get_coffee_machine_layout().size());
}

TEST_F(InterfaceTest, RepliesAreInitialized) {
  ASSERT_EQ(interface_->replies().size(), get_coffee_machine_layout().size());
}

TEST_F(InterfaceTest, DataIsInitialized) {
  ASSERT_EQ(interface_->data().commands.data(), interface_->commands().data());
  ASSERT_EQ(interface_->data().replies.data(), interface_->replies().data());
}

TEST_F(InterfaceTest, WriteStopCommands) {
  for (auto& command : interface_->commands()) {
    command.mode = moteus::Mode::kFault;
  }
  interface_->write_stop_commands();
  for (const auto& command : interface_->commands()) {
    ASSERT_EQ(command.mode, moteus::Mode::kStopped);
  }
}

TEST_F(InterfaceTest, ExpectFillsDictionaryKeys) {
  Dictionary action;
  interface_->initialize_action(action);
  ASSERT_TRUE(action.has("servo"));
  const Dictionary& servo = action("servo");
  for (const auto joint_name :
       {"left_grinder", "left_pump", "right_grinder", "right_pump"}) {
    ASSERT_TRUE(servo.has(joint_name));
    ASSERT_TRUE(servo(joint_name).has("position"));
    ASSERT_TRUE(servo(joint_name).has("velocity"));
    ASSERT_TRUE(servo(joint_name).has("kp_scale"));
    ASSERT_TRUE(servo(joint_name).has("kd_scale"));
    ASSERT_TRUE(servo(joint_name).has("maximum_torque"));
  }
}

TEST_F(InterfaceTest, DontThrowIfNoServoAction) {
  Dictionary action;
  ASSERT_NO_THROW(interface_->write_position_commands(action));
}

TEST_F(InterfaceTest, StopUnknownServos) {
  Dictionary action;
  action("servo")("left_pump")("position") = 0.0;
  action("servo")("left_grinder")("position") = 0.0;
  interface_->write_position_commands(action);
  ASSERT_EQ(interface_->commands()[0].mode, moteus::Mode::kPosition);
  ASSERT_EQ(interface_->commands()[1].mode, moteus::Mode::kPosition);
  ASSERT_EQ(interface_->commands()[2].mode, moteus::Mode::kStopped);
  ASSERT_EQ(interface_->commands()[3].mode, moteus::Mode::kStopped);
}

TEST_F(InterfaceTest, ForwardPositionCommands) {
  Dictionary action;
  action("servo")("left_grinder")("position") = 2 * M_PI;   // [rad]
  action("servo")("left_pump")("position") = M_PI;          // [rad]
  action("servo")("right_grinder")("position") = 2 * M_PI;  // [rad]
  action("servo")("right_pump")("position") = M_PI;         // [rad]

  interface_->write_position_commands(action);
  for (const auto& command : interface_->commands()) {
    ASSERT_EQ(command.mode, moteus::Mode::kPosition);
  }
  const auto& commands = interface_->commands();
  ASSERT_DOUBLE_EQ(commands[0].position.position, 1.0);  // [rev]
  ASSERT_DOUBLE_EQ(commands[1].position.position, 0.5);  // [rev]
  ASSERT_DOUBLE_EQ(commands[2].position.position, 1.0);  // [rev]
  ASSERT_DOUBLE_EQ(commands[3].position.position, 0.5);  // [rev]
}

TEST_F(InterfaceTest, StopServoIfNoAction) {
  Dictionary action;
  interface_->write_position_commands(action);
  ASSERT_EQ(interface_->commands()[0].mode, moteus::Mode::kStopped);
}

TEST_F(InterfaceTest, StopServoIfNoPosition) {
  Dictionary action;
  action("servo")("left_pump")("velocity") = 1.5;  // velocity but not position
  interface_->write_position_commands(action);
  ASSERT_EQ(interface_->commands()[0].mode, moteus::Mode::kStopped);
}

TEST_F(InterfaceTest, ForwardVelocityCommands) {
  Dictionary action;
  for (auto servo_name :
       {"left_pump", "left_grinder", "right_pump", "right_grinder"}) {
    action("servo")(servo_name)("position") = 2 * M_PI;  // [rad]
    action("servo")(servo_name)("velocity") = M_PI;      // [rad] / [s]
  }
  interface_->write_position_commands(action);
  for (const auto& command : interface_->commands()) {
    ASSERT_EQ(command.mode, moteus::Mode::kPosition);
    ASSERT_EQ(command.position.position, 1.0);
    ASSERT_EQ(command.position.velocity, 0.5);
  }
}

}  // namespace upkie
