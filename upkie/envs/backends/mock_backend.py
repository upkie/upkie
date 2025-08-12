#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path
from typing import Optional

import numpy as np

from upkie.utils.joystick import Joystick
from upkie.utils.robot_state import RobotState

from .backend import Backend


class MockBackend(Backend):
    r"""!
    Backend that mimics perfect commands for testing.
    """

    def __init__(self, js_path: str = "/dev/input/js0") -> None:
        r"""!
        Initialize mock backend.

        \param js_path Path to joystick device. Defaults to "/dev/input/js0".
        """
        spine_observation = {
            "base_orientation": {
                "pitch": 0.1,
                "angular_velocity": [-2e-3, 3e2, 1e-8],
                "linear_velocity": [1e3, 2e2, 3e1],
            },
            "floor_contact": {
                "contact": True,
            },
            "imu": {
                "orientation": [1.0, 0.0, 0.0, 0.0],
                "angular_velocity": [0.0, 0.0, 0.0],
                "linear_acceleration": [0.0, 0.0, 0.0],
            },
            "number": 0,
            "servo": {
                f"{side}_{joint}": {
                    "position": 0.0,
                    "velocity": 0.0,
                    "torque": 0.0,
                    "temperature": 42.0,
                    "voltage": 18.0,
                }
                for side in ("left", "right")
                for joint in ("hip", "knee", "wheel")
            },
            "wheel_odometry": {
                "position": 0.0,
                "velocity": 0.0,
            },
        }

        js_path = Path(js_path)
        joystick = Joystick(js_path) if js_path.exists() else None

        self.__spine_observation = spine_observation
        self.joystick = joystick

    def close(self) -> None:
        """!
        Close the mock backend (no-op).
        """
        pass

    def reset(self, init_state: Optional[RobotState] = None) -> dict:
        r"""!
        Reset the mock backend and get an initial observation.

        \param init_state Initial state of the robot (ignored in mock).
        \return Initial spine observation dictionary.
        """
        return self.get_spine_observation()

    def step(self, action: dict) -> dict:
        r"""!
        Apply action and step the mock backend.

        \param action Action dictionary in spine format.
        \return Spine observation dictionary after the step.
        """
        # Update internal spine observation from action
        servo_actions = action.get("servo", {})
        for joint_name, servo_action in servo_actions.items():
            if joint_name not in self.__spine_observation["servo"]:
                continue
            obs = self.__spine_observation["servo"][joint_name]
            for key in ("position", "velocity", "torque"):
                if key in servo_action and not np.isnan(servo_action[key]):
                    obs[key] = servo_action[key]

        if self.joystick is not None:
            self.joystick.write(self.__spine_observation)

        return self.get_spine_observation()

    def get_spine_observation(self) -> dict:
        r"""!
        Get observation in spine format from mock backend.
        """
        return self.__spine_observation
