// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#pragma once

#include <memory>
#include <string>

#include "upkie/cpp/controllers/ControllerPipeline.h"
#include "upkie/cpp/controllers/WheelBalancer.h"
#include "upkie/cpp/controllers/WheelStopper.h"
#include "upkie/cpp/exceptions/UpkieError.h"

namespace spines::common {

using upkie::cpp::controllers::ControllerPipeline;
using upkie::cpp::controllers::WheelBalancer;
using upkie::cpp::controllers::WheelStopper;
using upkie::cpp::exceptions::UpkieError;

/*! Prepare the controller pipeline.
 *
 * \param[in] pipeline Name of the controller pipeline to make.
 */
inline ControllerPipeline make_controllers(const std::string& pipeline,
                                           const double spine_frequency) {
  ControllerPipeline controllers;

  if (pipeline == "wheel_balancer") {
    spdlog::info("Initializing the \"wheel_balancer\" controller pipeline");
    auto wheel_stopper = std::make_shared<WheelStopper>();
    controllers.append(wheel_stopper);

    WheelBalancer::Parameters balancer_params;
    balancer_params.dt = 1.0 / spine_frequency;
    balancer_params.wheel_radius = 0.06;  // [m]
    auto balancer = std::make_shared<WheelBalancer>(balancer_params);
    controllers.append(balancer);
  } else if (pipeline != "servo") {
    spdlog::error("Unknown controller pipeline: \"{}\"", pipeline);
    throw UpkieError("Unknown controller pipeline: \"" + pipeline + "\"");
  }

  return controllers;
}

}  // namespace spines::common
