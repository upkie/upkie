#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
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

import numpy as np


class MockSpine:
    def __init__(self):
        self.observation = {
            "number": 0,
            "servo": {
                f"{side}_{joint}": {
                    "position": 0.0,
                    "velocity": 0.0,
                    "torque": 0.0,
                }
                for side in ("left", "right")
                for joint in ("hip", "knee", "wheel")
            },
            "imu": {
                "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
                "angular_velocity": np.zeros(3),
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
