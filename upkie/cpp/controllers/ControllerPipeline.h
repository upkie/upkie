// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#pragma once

#include <memory>
#include <vector>

#include "upkie/cpp/controllers/Controller.h"

//! Control.
namespace upkie::cpp::controllers {

/*! Controller pipeline.
 *
 * A controller pipeline is a list of controllers, to be executed in the order
 * they were appended. Controllers further down the pipeline may depend on the
 * actions written by those that precede them, which are written to the action
 * dictionary.
 */
class ControllerPipeline {
  using Dictionary = palimpsest::Dictionary;

 public:
  /*! Reset controllers.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config);

  /*! Append a controller at the end of the pipeline.
   *
   * \param controller New controller to append.
   */
  void append_controller(std::shared_ptr<Controller> controller) {
    controllers_.push_back(std::shared_ptr<Controller>(controller));
  }

  //! Controllers of the pipeline. Order matters.
  std::vector<std::shared_ptr<Controller>>& controllers() {
    return controllers_;
  }

  /*! Run controller pipeline.
   *
   * \param[in] observation Observation dictionary.
   * \param[in, out] action Action dictionary.
   */
  void run(const Dictionary& observation, Dictionary& action);

 private:
  //! Controller of the pipeline. Order matters.
  std::vector<std::shared_ptr<Controller>> controllers_;
};

}  // namespace upkie::cpp::controllers
