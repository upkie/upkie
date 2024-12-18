// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/Interface.h"
#include "upkie/cpp/exceptions/PositionCommandError.h"

namespace upkie::cpp::actuation {

using exceptions::PositionCommandError;

inline const std::vector<std::string> joint_names() noexcept {
  return {
      "left_hip",  "left_knee",  "left_wheel",
      "right_hip", "right_knee", "right_wheel",
  };
}

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
  void SetUp() override { interface_ = std::make_unique<SmallInterface>(); }

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
  ASSERT_EQ(interface_->commands().size(), 6);
}

TEST_F(InterfaceTest, RepliesAreInitialized) {
  ASSERT_EQ(interface_->replies().size(), 6);
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
  interface_->reset_action(action);
  ASSERT_TRUE(action.has("servo"));
  const Dictionary& servo = action("servo");
  for (const auto& joint_name : joint_names()) {
    ASSERT_TRUE(servo.has(joint_name));
    ASSERT_TRUE(servo(joint_name).has("position"));
    ASSERT_TRUE(servo(joint_name).has("velocity"));
    ASSERT_TRUE(servo(joint_name).has("kp_scale"));
    ASSERT_TRUE(servo(joint_name).has("kd_scale"));
    ASSERT_TRUE(servo(joint_name).has("maximum_torque"));
  }
}

TEST_F(InterfaceTest, ThrowIfMissingServoAction) {
  Dictionary action;
  action("servo")("left_hip")("position") = 0.0;
  action("servo")("left_knee")("position") = 0.0;
  // nothing for "right_hip" orr "right_knee"
  ASSERT_THROW(interface_->write_position_commands(action),
               PositionCommandError);
}

TEST_F(InterfaceTest, ForwardPositionCommands) {
  Dictionary action;
  action("servo")("left_hip")("position") = M_PI;        // [rad]
  action("servo")("left_knee")("position") = 2 * M_PI;   // [rad]
  action("servo")("left_wheel")("position") = 0.0;       // [rad]
  action("servo")("right_hip")("position") = M_PI;       // [rad]
  action("servo")("right_knee")("position") = 2 * M_PI;  // [rad]
  action("servo")("right_wheel")("position") = 0.0;      // [rad]

  interface_->write_position_commands(action);
  for (const auto& command : interface_->commands()) {
    ASSERT_EQ(command.mode, moteus::Mode::kPosition);
  }
  const auto& commands = interface_->commands();
  ASSERT_DOUBLE_EQ(commands[0].position.position, 0.5);  // [rev]
  ASSERT_DOUBLE_EQ(commands[1].position.position, 1.0);  // [rev]
  ASSERT_DOUBLE_EQ(commands[2].position.position, 0.0);  // [rev]
  ASSERT_DOUBLE_EQ(commands[3].position.position, 0.5);  // [rev]
  ASSERT_DOUBLE_EQ(commands[4].position.position, 1.0);  // [rev]
  ASSERT_DOUBLE_EQ(commands[5].position.position, 0.0);  // [rev]
}

TEST_F(InterfaceTest, SkipIfNoActionAtAll) {
  Dictionary action;
  ASSERT_NO_THROW(interface_->write_position_commands(action));
  ASSERT_EQ(interface_->commands()[0].mode, moteus::Mode::kStopped);
  ASSERT_EQ(interface_->commands()[1].mode, moteus::Mode::kStopped);
  ASSERT_EQ(interface_->commands()[2].mode, moteus::Mode::kStopped);
  ASSERT_EQ(interface_->commands()[3].mode, moteus::Mode::kStopped);
  ASSERT_EQ(interface_->commands()[4].mode, moteus::Mode::kStopped);
  ASSERT_EQ(interface_->commands()[5].mode, moteus::Mode::kStopped);
}

TEST_F(InterfaceTest, ThrowIfNoPosition) {
  Dictionary action;
  // all servos, but without any position command
  action("servo")("left_hip")("velocity") = 1.5;
  action("servo")("left_knee")("velocity") = 1.5;
  action("servo")("left_wheel")("velocity") = 1.5;
  action("servo")("right_hip")("velocity") = 1.5;
  action("servo")("right_knee")("velocity") = 1.5;
  action("servo")("right_wheel")("velocity") = 1.5;
  ASSERT_THROW(interface_->write_position_commands(action),
               PositionCommandError);
}

TEST_F(InterfaceTest, ForwardVelocityCommands) {
  Dictionary action;
  for (const auto servo_name : joint_names()) {
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

TEST_F(InterfaceTest, MaximumTorques) {
  Dictionary action;
  constexpr double kFeasibleTorque = 0.5;        // [N m]
  constexpr double kUnfeasisbleTorque = 1000.0;  // [N m]
  for (auto servo_name : joint_names()) {
    action("servo")(servo_name)("position") = 0.0;  // [rad]
    action("servo")(servo_name)("maximum_torque") = kFeasibleTorque;
  }
  interface_->write_position_commands(action);
  for (const auto& command : interface_->commands()) {
    ASSERT_EQ(command.mode, moteus::Mode::kPosition);
    ASSERT_EQ(command.position.maximum_torque, kFeasibleTorque);
  }
  for (auto servo_name : joint_names()) {
    action("servo")(servo_name)("maximum_torque") = kUnfeasisbleTorque;
    ASSERT_THROW(interface_->write_position_commands(action),
                 PositionCommandError);
    action("servo")(servo_name)("maximum_torque") = kFeasibleTorque;
    ASSERT_NO_THROW(interface_->write_position_commands(action));
  }
}

}  // namespace upkie::cpp::actuation
