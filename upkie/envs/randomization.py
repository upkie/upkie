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

from dataclasses import dataclass

import gin


@gin.configurable
@dataclass
class Randomization:
    reset_roll: float = 0.0
    reset_pitch: float = 0.0
    reset_x: float = 0.0
    reset_z: float = 0.0

    def update(
        self,
        reset_roll: float = 0.0,
        reset_pitch: float = 0.0,
        reset_x: float = 0.0,
        reset_z: float = 0.0,
    ) -> None:
        self.reset_roll = reset_roll
        self.reset_pitch = reset_pitch
        self.reset_x = reset_x
        self.reset_z = reset_z
