// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     SPDX-License-Identifier: Apache-2.0
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 */

#include "upkie/cpp/actuation/Pi3HatInterface.h"

namespace upkie::actuation {

Pi3HatInterface::Pi3HatInterface(const ServoLayout& layout, const int can_cpu,
                                 const Pi3Hat::Configuration& pi3hat_config)
    : Interface(layout),
      can_cpu_(can_cpu),
      pi3hat_config_(pi3hat_config),
      can_thread_(std::bind(&Pi3HatInterface::run_can_thread, this)) {}

Pi3HatInterface::~Pi3HatInterface() {
  done_ = true;  // comes first
  if (ongoing_can_cycle_) {
    // thread will exit at the end of its loop because done_ is set
    spdlog::info("Waiting for CAN thread to finish active cycle...");
    can_thread_.join();
    spdlog::info("CAN thread finished last cycle cleanly");
  } else /* thread is waiting for notification */ {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      can_wait_condition_.notify_one();
    }
    can_thread_.join();
  }
}

void Pi3HatInterface::reset(const Dictionary& config) {}

void Pi3HatInterface::observe(Dictionary& observation) const {
  ImuData imu_data;
  imu_data.orientation_imu_in_ars = get_attitude();
  imu_data.angular_velocity_imu_in_imu = get_angular_velocity();
  imu_data.linear_acceleration_imu_in_imu = get_linear_acceleration();

  // Eigen quaternions are serialized as [w, x, y, z]
  // See include/palimpsest/mpack/eigen.h in palimpsest
  observation("imu")("orientation") = imu_data.orientation_imu_in_ars;
  observation("imu")("angular_velocity") = imu_data.angular_velocity_imu_in_imu;
  observation("imu")("linear_acceleration") =
      imu_data.linear_acceleration_imu_in_imu;
}

void Pi3HatInterface::process_action(const Dictionary& action) {}

void Pi3HatInterface::cycle(
    std::function<void(const moteus::Output&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (ongoing_can_cycle_) {
    throw std::logic_error(
        "Cycle cannot be called before the previous one has completed.");
  }

  callback_ = std::move(callback);
  ongoing_can_cycle_ = true;

  can_wait_condition_.notify_all();
}

void Pi3HatInterface::run_can_thread() {
  upkie::utils::configure_cpu(can_cpu_);
  upkie::utils::configure_scheduler(10);
  pi3hat_.reset(new Pi3Hat({pi3hat_config_}));
  pthread_setname_np(pthread_self(), "can_thread");
  while (!done_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!ongoing_can_cycle_) {
        can_wait_condition_.wait(lock);
        if (done_) {
          return;
        }
        if (!ongoing_can_cycle_) {
          continue;
        }
      }
    }
    auto output = cycle_can_thread();
    std::function<void(const moteus::Output&)> callback_copy;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ongoing_can_cycle_ = false;
      std::swap(callback_copy, callback_);
    }
    callback_copy(output);
  }
}

moteus::Output Pi3HatInterface::cycle_can_thread() {
  tx_can_.resize(data_.commands.size());
  int out_idx = 0;
  for (const auto& cmd : data_.commands) {
    const auto& query = cmd.query;

    auto& can = tx_can_[out_idx++];

    can.expect_reply = query.any_set();
    can.id = cmd.id | (can.expect_reply ? 0x8000 : 0x0000);
    can.size = 0;

    can.bus = [&]() {
      const auto it = servo_bus_map().find(cmd.id);
      if (it == servo_bus_map().end()) {
        return 1;
      }
      return it->second;
    }();

    moteus::WriteCanFrame write_frame(can.data, &can.size);
    switch (cmd.mode) {
      case moteus::Mode::kStopped: {
        moteus::EmitStopCommand(&write_frame);
        break;
      }
      case moteus::Mode::kPosition:
      case moteus::Mode::kZeroVelocity: {
        moteus::EmitPositionCommand(&write_frame, cmd.position, cmd.resolution);
        break;
      }
      default: {
        throw std::logic_error("unsupported mode");
      }
    }
    moteus::EmitQueryCommand(&write_frame, cmd.query);
  }

  rx_can_.resize(data_.commands.size() * 2);

  Pi3Hat::Input input;
  input.tx_can = {tx_can_.data(), tx_can_.size()};
  input.rx_can = {rx_can_.data(), rx_can_.size()};
  input.attitude = &attitude_;
  input.request_attitude = true;
  input.wait_for_attitude = true;  // we may revisit this later

  moteus::Output result;
  const auto pi3hat_output = pi3hat_->Cycle(input);
  if (pi3hat_output.error) {
    spdlog::error("pi3hat: {}", pi3hat_output.error);
  }
  for (size_t i = 0; i < pi3hat_output.rx_can_size && i < data_.replies.size();
       ++i) {
    const auto& can = rx_can_[i];
    data_.replies[i].id = (can.id & 0x7f00) >> 8;
    data_.replies[i].result = moteus::ParseQueryResult(can.data, can.size);
    result.query_result_size = i + 1;
  }
  if (!pi3hat_output.attitude_present) {  // because we wait for attitude
    spdlog::warn("Missing attitude data!");
  }
  return result;
}

}  // namespace upkie::actuation
