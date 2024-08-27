#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria


class MockSpine:
    def __init__(self):
        self.observation = {
            "base_orientation": {
                "pitch": 0.1,
                "angular_velocity": [-2e-3, 3e2, 1e-8],
                "linear_velocity": [1e3, 2e2, 3e1],
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

    def start(self, config: dict) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_action(self, action) -> None:
        self.action = action

    def get_observation(self) -> dict:
        self.observation["number"] += 1
        return self.observation
