// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023 Inria

#include "upkie/cpp/observers/BaseOrientation.h"
#include "upkie/cpp/observers/FloorContact.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/observers/WheelOdometry.h"

namespace spines::common {

using upkie::cpp::observers::BaseOrientation;
using upkie::cpp::observers::FloorContact;
using upkie::cpp::observers::ObserverPipeline;
using upkie::cpp::observers::WheelOdometry;

/*! Prepare the controller pipeline.
 *
 * \param[in] spine_frequency Spine frequency in [Hz].
 */
inline ObserverPipeline make_observers(unsigned spine_frequency) {
  ObserverPipeline observation;

  BaseOrientation::Parameters base_orientation_params;
  auto base_orientation =
      std::make_shared<BaseOrientation>(base_orientation_params);
  observation.append_observer(base_orientation);

  FloorContact::Parameters floor_contact_params;
  floor_contact_params.dt = 1.0 / spine_frequency;
  auto floor_contact = std::make_shared<FloorContact>(floor_contact_params);
  observation.append_observer(floor_contact);

  WheelOdometry::Parameters odometry_params;
  odometry_params.dt = 1.0 / spine_frequency;
  auto odometry = std::make_shared<WheelOdometry>(odometry_params);
  observation.append_observer(odometry);

  return observation;
}

}  // namespace spines::common
