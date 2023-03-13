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

"""Upkie wheeled biped bending its knees."""

import meshcat_shapes
import numpy as np
import pink
import qpsolvers
from loop_rate_limiters import RateLimiter
from pink import solve_ik
from pink.tasks import BodyTask, PostureTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer

from agents.pink_balancer.whole_body_controller import WholeBodyController

if __name__ == "__main__":
    config = {}
    controller = WholeBodyController(
        config,
        gain_scale=1.0,
        max_crouch_height=0.0,
        max_crouch_velocity=100.0,
        turning_gain_scale=1.0,
    )
