#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

import gin

from upkie.utils.clamp import clamp

from .height_controller import HeightController
from .wheel_controller import WheelController


@gin.configurable
class WholeBodyController:
    r"""!
    Coordinate leg inverse kinematics and wheel balancing.
    """

    ## \var gain_scale
    ## PD gain scale for hip and knee joints.
    gain_scale: float

    ## \var turning_gain_scale
    ## Additional gain scale added when the robot is turning to keep the legs
    ## stiff while the ground pulls them apart.
    turning_gain_scale: float

    def __init__(
        self, gain_scale: float, turning_gain_scale: float, visualize: bool
    ):
        r"""!
        Create controller.

        \param gain_scale PD gain scale for hip and knee joints.
        \param turning_gain_scale Additional gain scale added when the robot
            is turning to keep the legs stiff in spite of the ground
            pulling them apart.
        \param visualize If true, open a MeshCat visualizer on the side.
        """
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.height_controller = HeightController(visualize=visualize)
        self.turning_gain_scale = turning_gain_scale
        self.wheel_controller = WheelController()

    def cycle(self, observation: dict, dt: float) -> dict:
        r"""!
        Compute action for a new cycle.

        \param observation Latest observation.
        \param dt Duration in seconds until next cycle.
        \return Dictionary with the new action and some logging.
        """
        leg_action = self.height_controller.cycle(observation, dt)
        wheel_action = self.wheel_controller.cycle(observation, dt)
        servo_action = {
            "left_hip": leg_action["servo"]["left_hip"],
            "left_knee": leg_action["servo"]["left_knee"],
            "left_wheel": wheel_action["servo"]["left_wheel"],
            "right_hip": leg_action["servo"]["right_hip"],
            "right_knee": leg_action["servo"]["right_knee"],
            "right_wheel": wheel_action["servo"]["right_wheel"],
        }
        turning_prob = self.wheel_controller.turning_probability
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            servo_action[joint_name]["kp_scale"] = kp_scale
            servo_action[joint_name]["kd_scale"] = kd_scale
        return {"servo": servo_action}
