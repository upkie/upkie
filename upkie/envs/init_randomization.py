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

"""!
Domain randomization of Upkie environments.
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation


@dataclass
class InitRandomization:

    """!
    Domain randomization parameters for Upkie's initial state.
    """

    roll: float = 0.0
    pitch: float = 0.0
    x: float = 0.0
    z: float = 0.0
    v_x: float = 0.0
    v_z: float = 0.0
    omega_x: float = 0.0
    omega_y: float = 0.0

    def sample_orientation(self, np_random) -> ScipyRotation:
        yaw_pitch_roll_bounds = np.array([0.0, self.pitch, self.roll])
        yaw_pitch_roll = np_random.uniform(
            low=-yaw_pitch_roll_bounds,
            high=+yaw_pitch_roll_bounds,
            size=3,
        )
        return ScipyRotation.from_euler("ZYX", yaw_pitch_roll)

    def sample_position(self, np_random) -> np.ndarray:
        default_position = np.array([0.0, 0.0, 0.6])
        return default_position + np_random.uniform(
            low=np.array([-self.x, 0.0, 0.0]),
            high=np.array([+self.x, 0.0, self.z]),
            size=3,
        )

    def sample_linear_velocity(self, np_random) -> np.ndarray:
        return np_random.uniform(
            low=np.array([-self.v_x, 0.0, -self.v_z]),
            high=np.array([+self.v_x, 0.0, +self.v_z]),
            size=3,
        )

    def sample_angular_velocity(self, np_random) -> np.ndarray:
        return np_random.uniform(
            low=np.array([-self.omega_x, -self.omega_y, 0.0]),
            high=np.array([+self.omega_x, +self.omega_y, 0.0]),
            size=3,
        )
