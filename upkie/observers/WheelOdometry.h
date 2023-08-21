/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <palimpsest/Dictionary.h>

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

#include "vulp/observation/Observer.h"

namespace upkie::observers {

using palimpsest::Dictionary;
using vulp::observation::Observer;

/*! Observe the relative motion of the floating base with respect to the floor.
 *
 * Consider a given wheel \f$i \in \{\mathit{left}, \mathit{right}\}\f$.
 * Assuming the wheel is rolling without slipping on the floor, its velocity is
 * related to that of the base, in the base frame \f$B\f$, by:
 *
 * \f[
 * \sideset{_B}{} {\begin{bmatrix} v_i^x \\ v_i^y \\ v_i^z \end{bmatrix}}
 * =
 * \sideset{_B}{} {\begin{bmatrix} v^x \\ v^y \\ 0 \end{bmatrix}}
 * +
 * \sideset{_B}{} {\begin{bmatrix} 0 \\ 0 \\ \omega \end{bmatrix}}
 * \times
 * \sideset{_B}{} {\begin{bmatrix} r_i^x \\ r_i^y \\ 0 \end{bmatrix}}
 * +
 * \sideset{_B}{} {\begin{bmatrix} \dot{r}_i^x \\ \dot{r}_i^y \\ \dot{r}_i^z
 * \end{bmatrix}} \f]
 *
 * where \f$v_i\f$ is the linear velocity of the contact point at wheel
 * \f$i\f$, \f$v\f$ is the linear velocity of the base, \f$\omega\f$ is the
 * body angular velocity of the base (in full: the magnitude of the angular
 * velocity from base to world in base, whose direction we assume is aligned
 * with the ground normal), \f$r_i\f$ is the position of the wheel contact
 * point in the base frame, and \f$\dot{r}_i\f$ its time derivative (zero if
 * the hips and knees don't move, non-zero if they do).
 *
 * \image html wheel-odometry.png
 * \image latex wheel-odometry.eps
 *
 * When we infer, all these equations should be satisfied simultaneously. The
 * system is over-determined so we can do it in a least-squares sense.
 *
 * In this observer, we further assume that \f$\omega\f$ and \f$\dot{r}_i\f$
 * are zero, so that the kinematics simplify to:
 *
 * \f[
 * \sideset{_B}{} {\begin{bmatrix} v_i^x \\ v_i^y \end{bmatrix}}
 * =
 * \sideset{_B}{} {\begin{bmatrix} v^x \\ v^y \end{bmatrix}}
 * \f]
 *
 * We then simply compute \f$v\f$ from \f$v_i\f$'s with an arithmetic average.
 * The relative position of the base with respect to the ground is finally
 * computed by forward integration:
 *
 * \f[
 * p(T) = \int_{t=0}^T v(t) {\rm d}t
 * \f]
 *
 * The initial time \f$t = 0\f$ in this formula corresponds to touchdown.
 * Odometry is only incremented when at least one wheel is in contact with the
 * ground; its output is stationary when both legs are in the air.
 */
class WheelOdometry : public Observer {
 public:
  //! Observer parameters.
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      if (!config.has("wheel_odometry")) {
        spdlog::debug("No \"wheel_odometry\" runtime configuration");
        return;
      }

      spdlog::info("Applying \"wheel_odometry\" runtime configuration");
      if (config("wheel_odometry").has("signed_radius")) {
        const auto& signed_radius_dict =
            config("wheel_odometry")("signed_radius");
        signed_radius.clear();
        for (const auto& key : signed_radius_dict.keys()) {
          signed_radius[key] = signed_radius_dict(key);
        }
      }
    }

    //! Spine timestep in [s]
    double dt;

    //! Signed radius in [m] for each wheel
    std::map<std::string, double> signed_radius;
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit WheelOdometry(const Parameters& params);

  /*! Observe leg contacts.
   *
   * \param[in] observation Dictionary to read other observations from.
   */
  void read(const Dictionary& observation) final;

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return "wheel_odometry"; }

  /*! Reset observer.
   *
   * \param[in] config Global configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final;

 private:
  /*! Compute average linear velocity from wheels in contact.
   *
   * \param[in] floor_contact Wheel contact states.
   * \param[in] servo Wheel servo readings.
   *
   * \return Average linear velocity in [m] / [s].
   */
  double compute_average_velocity(const Dictionary& floor_contact,
                                  const Dictionary& servo);

 private:
  //! Observer parameters.
  Parameters params_;

  /*! Sagittal ground position in [m].
   *
   * Its origin is defined the first time the robot hits the ground.
   */
  double position_;

  //! Sagittal ground velocity in [m] / [s].
  double velocity_;
};

}  // namespace upkie::observers
