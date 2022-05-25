#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any, Dict, Tuple

import gin
import numpy as np

from agents.blue_balancer.wheel_balancer import WheelBalancer
from utils.clamp import clamp, clamp_abs


def observe(observation, configuration, servo_layout) -> np.ndarray:
    """
    Compute configuration vector from a new observation.

    Args:
        observation: Observation dictionary.
        configuration: Previous configuration.
        servo_layout: Robot servo layout.

    Returns:
        Configuration vector from observation.
    """
    q = configuration.q.copy()
    for joint, servo in servo_layout.items():
        if "configuration_index" not in servo:
            continue
        i = servo["configuration_index"]
        q[i] = observation["servo"][joint]["position"]
        # tangent_index = configuration_index - 1
        # v[tangent_index] = observation["servo"][joint]["velocity"]
    return q


@gin.configurable
def forward_kinematics(
    q_hip: float, q_knee: float, limb_length: float
) -> float:
    """
    Compute forward kinematics for a single leg.

    Args:
        q_hip: Angle of the hip joint, in radians.
        q_knee: Angle of the knee joint, in radians.
        limb_length: (Model) Length of both links of the leg.

    Returns:
        Crouch distance (positive, zero for extended legs) in meters.

    The derivation of this function is documented in `Kinematics of a
    symmetric leg`_.

    .. _`Kinematics of a symmetric leg`:
        https://scaron.info/blog/kinematics-of-a-symmetric-leg.html
    """
    height = limb_length * (np.cos(q_hip) + np.cos(q_hip + q_knee))
    leg_length = 2.0 * limb_length
    crouch = leg_length - height
    return crouch


@gin.configurable
def inverse_kinematics(
    crouch: float,
    crouch_velocity: float,
    limb_length: float,
    velocity_limit: float,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Solve inverse kinematics for a single leg.

    Args:
        crouch: Crouch distance (positive, zero for extended legs) in meters.
        crouch_velocity: Time derivative of the crouch, in m / s.
        limb_length: (Model) Length of both links of the leg.
        velocity_limit: (Model) Maximum joint velocity in rad / s.

    Returns:
        ===========  =========================================================
        ``q_hip``     Angle of the hip joint in radians.
        ``q_knee``    Angle of the knee joint in radians.
        ``v_hip``     Angular velocity for the hip joint in rad / s.
        ``v_knee``    Angular velocity for the knee joint in rad / s.
        ===========  =========================================================

    The derivation of this function is documented in `Kinematics of a
    symmetric leg`_.

    .. _`Kinematics of a symmetric leg`:
        https://scaron.info/blog/kinematics-of-a-symmetric-leg.html
    """
    leg_length = 2.0 * limb_length
    assert 0 < crouch < leg_length, "Leg is under- or over-extended"
    height = leg_length - crouch
    height_velocity = -crouch_velocity
    x = height / leg_length
    q_hip = np.arccos(x)
    v_hip = -height_velocity / np.sqrt(leg_length ** 2 - height ** 2)
    v_hip = clamp_abs(v_hip, velocity_limit)
    q_knee = -2.0 * q_hip
    v_knee = -2.0 * v_hip
    v_knee = clamp_abs(v_knee, velocity_limit)
    return (q_hip, q_knee), (v_hip, v_knee)


@gin.configurable
class WholeBodyController:

    """
    Coordinate inverse kinematics and wheel balancing.

    Attributes:
        gain_scale: PD gain scale for hip and knee joints.
        max_crouch: Maximum distance along the vertical axis that the
            robot goes down while crouching, in meters.
        max_crouch_velocity: Maximum vertical velocity in m / s.
        position_right_in_left: Translation from the left contact frame to
            the right contact frame, expressed in the left contact frame.
        target_crouch: Target vertical distance in meters by which to crouch
            (equivalently: lift the wheels), with respect to the initial
            configuration where the legs are extended. The target height (e.g.
            of the COM) above ground is therefore equal to ``maximum_height -
            target_crouch``.
        transform_rest_to_world: Rest frame pose for each end effector.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    gain_scale: float
    max_crouch: float
    max_crouch_velocity: float
    target_crouch: float
    turning_gain_scale: float

    def __init__(
        self,
        config: Dict[str, Any],
        gain_scale: float,
        max_crouch: float,
        max_crouch_velocity: float,
        turning_gain_scale: float,
        wheel_distance: float,
    ):
        """
        Create controller.

        Args:
            config: Global configuration dictionary.
            gain_scale: PD gain scale for hip and knee joints.
            max_crouch: Maximum distance along the vertical axis that the robot
                goes down while crouching, in meters.
            max_crouch_velocity: Maximum vertical velocity in [m] / [s].
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
            wheel_distance: Lateral distance between the two wheels in meters.
                This controller does not handle the case where the two wheels
                are not in the lateral plane.
        """
        joint_names = [
            f"{side}_{joint}"
            for side in ["left", "right"]
            for joint in ["hip", "knee", "wheel"]
        ]
        servo_action = {
            joint: {
                "position": np.nan,
                "velocity": np.nan,
            }
            for joint in joint_names
        }
        self.crouch = np.nan
        self.crouch_velocity = np.nan
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.max_crouch = max_crouch
        self.max_crouch_velocity = max_crouch_velocity
        self.position_right_in_left = np.ndarray([0.0, wheel_distance, 0.0])
        self.servo_action = servo_action
        self.servo_layout = config["servo_layout"]
        self.target_crouch = 0.0
        self.turning_gain_scale = turning_gain_scale
        self.wheel_balancer = WheelBalancer()  # type: ignore

    def update_target_crouch(
        self, observation: Dict[str, Any], dt: float
    ) -> None:
        """
        Update target crouch from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        try:
            axis_value: float = observation["joystick"]["pad_axis"][1]
            velocity = self.max_crouch_velocity * axis_value
        except KeyError:
            velocity = 0.0
        crouch = self.target_crouch
        crouch += velocity * dt
        self.target_crouch = clamp(crouch, 0.0, self.max_crouch)
        self.target_crouch_velocity = velocity

    def _initialize_crouch(self, observation: Dict[str, Any]) -> None:
        """
        Initialize crouch and its derivative from first observation.

        Args:
            observation: Observation from the spine.
        """
        q_hip = np.mean(
            observation["servo"][hip]["position"]
            for hip in ["left_hip", "right_hip"]
        )
        q_knee = np.mean(
            observation["servo"][knee]["position"]
            for knee in ["left_knee", "right_knee"]
        )
        self.crouch = forward_kinematics(q_hip, q_knee)
        self.crouch_velocity = 0.0

    def cycle(self, observation: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """
        Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        if np.isnan(self.crouch):
            self._initialize_crouch(observation)

        # TODO(scaron): velocities
        self.update_target_crouch(observation, dt)
        q, v = inverse_kinematics(self.crouch, self.crouch_velocity)
        self.servo_action["left_hip"]["position"] = q[0]
        self.servo_action["left_knee"]["position"] = q[1]
        self.servo_action["right_hip"]["position"] = q[0]
        self.servo_action["right_knee"]["position"] = q[1]

        self.wheel_balancer.cycle(observation, dt)
        (
            left_wheel_velocity,
            right_wheel_velocity,
        ) = self.wheel_balancer.get_wheel_velocities(
            self.position_right_in_left
        )

        self.servo_action["left_wheel"] = {
            "position": np.nan,
            "velocity": left_wheel_velocity,
        }
        self.servo_action["right_wheel"] = {
            "position": np.nan,
            "velocity": right_wheel_velocity,
        }

        turning_prob = self.wheel_balancer.turning_probability
        # using the same numbers for both gain scales for now
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            self.servo_action[joint_name]["kp_scale"] = kp_scale
            self.servo_action[joint_name]["kd_scale"] = kd_scale

        return {
            "servo": self.servo_action,
            "wheel_balancer": self.wheel_balancer.log(),
        }
