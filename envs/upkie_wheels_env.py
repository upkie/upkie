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

import gin
import gym


@gin.configurable
class UpkieWheelsEnv(gym.Env):

    def id(self) -> str:
        """
        Name and version of this environment for registration.

        Returns:
            Name and version of the environment.
        """
        return f"UpkieWheelsEnv-v{self.version}"

    @staticmethod
    def gin_config():
        """
        Path to the Gin configuration for this environment.
        """
        dirname = path.dirname(__file__)
        basename = path.basename(__file__).replace(".py", ".gin")
        return f"{dirname}/{basename}"

    def detect_fall(self, pitch: float) -> bool:
        """
        Detect a fall based on the body-to-world pitch angle.

        Args:
            pitch: Current pitch angle in [rad].

        Returns:
            True if and only if a fall is detected.
        """
        return abs(pitch) > self.fall_pitch
