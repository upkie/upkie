#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import gin
import numpy as np
from wheel_controller import WheelController
import math

from scipy.spatial.transform import Rotation as R
 
def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler


from upkie.utils.clamp import clamp

M_PI = 3.14159265358979323846

@gin.configurable
class ServoController:
    """!
    Balance Upkie using its wheels.
    """

    gain_scale: float
    turning_gain_scale: float

    def __init__(
        self,
        gain_scale: float,
        turning_gain_scale: float,
        wheel_distance: float,
    ):
        """!
        Create controller.

        @param gain_scale PD gain scale for hip and knee joints.
        @param turning_gain_scale Additional gain scale added when the robot is
            turning to keep the legs stiff in spite of the ground pulling them
            apart.
        @param wheel_distance Lateral distance between the two wheels in
            meters. This controller does not handle the case where the two
            wheels are not in the lateral plane.
        """
        ## PD gain scale for hip and knee joints.
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)

        ## Translation from the left contact frame to the right contact frame,
        ## expressed in the left contact frame.
        self.position_right_in_left = np.array([0.0, wheel_distance, 0.0])

        ## Additional gain scale added when the robot is turning to keep the
        ## legs stiff while the ground pulls them apart.
        self.turning_gain_scale = turning_gain_scale

        self.__servo_action = None
        self.__wheel_balancer = WheelController()  # type: ignore
        self.__pos_left = 0.0
        self.__pos_right = 0.0

    @property
    def wheel_radius(self) -> float:
        """!Wheel radius in [m]."""
        return self.__wheel_balancer.wheel_radius

    def initialize_servo_action(self, observation: dict) -> None:
        """!
        Initialize default servo action from initial observation.

        @param observation Initial observation.
        """
        self.__servo_action = {
            wheel: {
                "position": 0.0,
                "velocity": 0.0,
                "maximum_torque": 1.0,  # mj5208 actuators
            }
            for wheel in ("left_wheel", "right_wheel")
        }


    def cycle(self, observation: dict, dt: float) -> dict:
        """!
        Compute action for a new cycle.

        @param observation Latest observation.
        @param dt Duration in seconds until next cycle.
        @return Dictionary with the new action and some internal state for
            logging.
        """
        if self.__servo_action is None:
            self.initialize_servo_action(observation)

        print("\r",observation["imu"]["orientation"], "\t")
        print("\r",observation["imu"]["angular_velocity"],"\t")
        print("\r",observation["imu"]["linear_acceleration"], "\t")
        
        euler = quaternion2euler(observation["imu"]["orientation"])
        pitch = euler[1]
        
        torch = (pitch + 3.22822) * 8
        
        self.__servo_action.update({
            "left_wheel": {
                "position": self.__pos_left * M_PI,
                "velocity": 0.0,
                "feedforward_torque":torch
            },
            "right_wheel":{
                "position": self.__pos_right * M_PI,
                "velocity": 0.0,
                "feedforward_torque":torch,
            }
        })
        
        return {
            "servo": self.__servo_action
        }
