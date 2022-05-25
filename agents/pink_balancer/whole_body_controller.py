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

from typing import Any, Dict

import gin
import numpy as np
import pink
import pink.models
import pinocchio as pin
import upkie_description
from agents.pink_balancer.wheel_balancer import WheelBalancer
from pink import solve_ik
from pink.tasks import BodyTask, PostureTask
from pink.utils import custom_configuration_vector
from utils.clamp import clamp


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


def serialize_to_servo_controller(
    configuration, velocity, servo_layout
) -> Dict[str, dict]:
    """
    Serialize robot state for the spine.

    Args:
        configuration: Robot configuration.
        velocity: Robot velocity in tangent space.
        servo_layout: Robot servo layout.

    Returns:
        Dictionary of position and velocity targets for each joint.
    """
    target = {}
    for joint, servo in servo_layout.items():
        if "configuration_index" not in servo:
            continue
        i_q = servo["configuration_index"]
        i_v = i_q - 1
        target[joint] = {"position": configuration.q[i_q]}
        target[joint]["velocity"] = velocity[i_v]
    return target


@gin.configurable
class WholeBodyController:

    """
    Coordinate inverse kinematics and wheel balancing.

    Attributes:
        crouch_velocity: Maximum vertical velocity in [m] / [s].
        gain_scale: PD gain scale for hip and knee joints.
        max_crouch_height: Maximum distance along the vertical axis that the
            robot goes down while crouching, in [m].
        robot: Robot model used for inverse kinematics.
        target_position_wheel_in_rest: Target position in the rest frame.
        tasks: Dictionary of inverse kinematics tasks.
        transform_rest_to_world: Rest frame pose for each end effector.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    crouch_velocity: float
    gain_scale: float
    max_crouch_height: float
    robot: pin.RobotWrapper
    target_position_wheel_in_rest: np.ndarray
    tasks: Dict[str, Any]
    transform_rest_to_world: Dict[str, np.ndarray]
    turning_gain_scale: float

    def __init__(
        self,
        config: Dict[str, Any],
        crouch_velocity: float,
        gain_scale: float,
        max_crouch_height: float,
        turning_gain_scale: float,
    ):
        """
        Initialize controller.

        Args:
            config: Global configuration dictionary.
            crouch_velocity: Maximum vertical velocity in [m] / [s].
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in [m].
            gain_scale: PD gain scale for hip and knee joints.
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
        """
        robot = pink.models.build_from_urdf(upkie_description.urdf_path)
        configuration = pink.apply_configuration(robot, robot.q0)
        tasks = {
            "base": BodyTask(
                "base",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            "left_contact": BodyTask(
                "left_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=1e2,
            ),
            "right_contact": BodyTask(
                "right_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=1e2,
            ),
            "posture": PostureTask(
                cost=1e-3,  # [cost] / [rad]
            ),
        }
        tasks["posture"].set_target(
            custom_configuration_vector(robot, left_knee=0.2, right_knee=-0.2)
        )
        self.configuration = configuration
        self.crouch_velocity = crouch_velocity
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.initialized = False
        self.max_crouch_height = max_crouch_height
        self.robot = robot
        self.servo_layout = config["servo_layout"]
        self.target_position_wheel_in_rest = np.zeros(3)
        self.target_offset = {
            "left_contact": np.zeros(3),
            "right_contact": np.zeros(3),
        }
        self.tasks = tasks
        self.transform_rest_to_world = {
            "left_contact": np.zeros((4, 4)),
            "right_contact": np.zeros((4, 4)),
        }
        self.wheel_balancer = WheelBalancer()  # type: ignore
        self.turning_gain_scale = turning_gain_scale
