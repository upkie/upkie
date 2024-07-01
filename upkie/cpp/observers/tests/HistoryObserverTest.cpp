// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include <palimpsest/Dictionary.h>
#include <palimpsest/exceptions/KeyError.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/observation/HistoryObserver.h"

namespace upkie::cpp::observation::tests {

using palimpsest::Dictionary;

TEST(HistoryObserver, DoubleInitialization) {
  HistoryObserver<double> history_observer(
      /* keys = */ std::vector<std::string>{"servo", "left_knee", "torque"},
      /* size = */ 3,
      /* default_value = */ -1.0);

  Dictionary observation;

  history_observer.write(observation);
  const auto& values = observation("history")("servo")("left_knee")("torque")
                           .as<std::vector<double>>();
  ASSERT_EQ(values.size(), 3);
  ASSERT_DOUBLE_EQ(values[0], -1.0);
  ASSERT_DOUBLE_EQ(values[1], -1.0);
  ASSERT_DOUBLE_EQ(values[2], -1.0);
}

TEST(HistoryObserver, DoubleReadThenWrite) {
  HistoryObserver<double> history_observer(
      /* keys = */ std::vector<std::string>{"servo", "left_knee", "torque"},
      /* size = */ 3,
      /* default_value = */ std::numeric_limits<double>::quiet_NaN());

  Dictionary observation;
  observation("servo")("left_knee")("torque") = 3.0;
  history_observer.read(observation);
  observation("servo")("left_knee")("torque") = 2.0;
  history_observer.read(observation);
  observation("servo")("left_knee")("torque") = 1.0;
  history_observer.read(observation);

  history_observer.write(observation);
  const auto& values = observation("history")("servo")("left_knee")("torque")
                           .as<std::vector<double>>();
  ASSERT_EQ(values.size(), 3);
  ASSERT_DOUBLE_EQ(values[0], 1.0);  // most recent comes first
  ASSERT_DOUBLE_EQ(values[1], 2.0);
  ASSERT_DOUBLE_EQ(values[2], 3.0);
}

TEST(HistoryObserver, Vector3dInitialization) {
  HistoryObserver<Eigen::Vector3d> history_observer(
      /* keys = */ std::vector<std::string>{"imu", "linear_acceleration"},
      /* size = */ 4,
      /* default_value = */ Eigen::Vector3d::Zero());

  Dictionary observation;
  history_observer.write(observation);
  const auto& values = observation("history")("imu")("linear_acceleration")
                           .as<std::vector<Eigen::Vector3d>>();
  ASSERT_EQ(values.size(), 4);
  for (unsigned i = 0; i < values.size(); ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      ASSERT_DOUBLE_EQ(values[i][j], 0.0);
    }
  }
}

TEST(HistoryObserver, Vector3dReadThenWrite) {
  HistoryObserver<Eigen::Vector3d> history_observer(
      /* keys = */ std::vector<std::string>{"imu", "linear_acceleration"},
      /* size = */ 3,
      /* default_value = */ Eigen::Vector3d::Zero());

  Dictionary observation;
  observation("imu").insert<Eigen::Vector3d>("linear_acceleration",
                                             Eigen::Vector3d::Zero());
  auto& linear_acceleration =
      observation("imu")("linear_acceleration").as<Eigen::Vector3d>();
  linear_acceleration.z() = 3.2;
  history_observer.read(observation);
  linear_acceleration.z() = 2.1;
  history_observer.read(observation);
  linear_acceleration.z() = 1.0;
  history_observer.read(observation);

  history_observer.write(observation);
  const auto& values = observation("history")("imu")("linear_acceleration")
                           .as<std::vector<Eigen::Vector3d>>();
  ASSERT_EQ(values.size(), 3);
  ASSERT_DOUBLE_EQ(values[0].z(), 1.0);
  ASSERT_DOUBLE_EQ(values[1].z(), 2.1);
  ASSERT_DOUBLE_EQ(values[2].z(), 3.2);
}

}  // namespace upkie::cpp::observation::tests
