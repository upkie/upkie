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


@dataclass
class InitRandomization:

    """!
    Domain randomization parameters for Upkie's initial state.
    """

    roll: float = 0.0
    pitch: float = 0.0
    x: float = 0.0
    z: float = 0.0

    def update(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        x: float = 0.0,
        z: float = 0.0,
    ) -> None:
        self.roll = roll
        self.pitch = pitch
        self.x = x
        self.z = z
