// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/interfaces/MockInterface.h"

using std::chrono::duration_cast;
using std::chrono::microseconds;

namespace upkie::cpp::interfaces {

MockInterface::MockInterface(const double dt) : Interface(), dt_(dt) {
  query_results_.clear();
  for (const auto& id_bus : servo_layout().servo_bus_map()) {
    const auto& servo_id = id_bus.first;
    moteus::QueryResult result;
    result.d_current = std::numeric_limits<double>::quiet_NaN();
    result.fault = 0;
    result.q_current = std::numeric_limits<double>::quiet_NaN();
    result.rezero_state = false;
    result.temperature = std::numeric_limits<double>::quiet_NaN();
    result.voltage = std::numeric_limits<double>::quiet_NaN();
    query_results_.try_emplace(servo_id, result);
  }
}

void MockInterface::reset(const Dictionary& config) {}

void MockInterface::observe(Dictionary& observation) const {
  Interface::observe_imu(observation);
}

void MockInterface::process_action(const Dictionary& action) {}

void MockInterface::cycle(std::function<void(const moteus::Output&)> callback) {
  const moteus::Data& data = this->data_;
  assert(data.replies.size() == data.commands.size());

  for (const auto& command : data.commands) {
    const auto servo_id = command.id;
    auto& result = query_results_[servo_id];
    result.mode = command.mode;
    const auto& target = command.position;
    if (std::isnan(target.position) && !std::isnan(target.velocity) &&
        !std::isnan(result.position)) {
      result.position += target.velocity * dt_;
    } else {
      result.position = target.position;
    }
    result.velocity = target.velocity;
    result.torque = target.feedforward_torque;
  }

  moteus::Output output;
  for (size_t i = 0; i < data.replies.size(); ++i) {
    const auto servo_id = data.commands[i].id;
    data.replies[i].id = servo_id;
    data.replies[i].result = query_results_[servo_id];
    output.query_result_size = i + 1;
  }
  callback(output);
}

}  // namespace upkie::cpp::interfaces
