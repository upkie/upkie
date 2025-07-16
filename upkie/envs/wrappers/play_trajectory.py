#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

from pathlib import Path
from typing import Tuple

import gymnasium as gym
import numpy as np
import pandas as pd

from upkie.envs import UpkieServos
from upkie.exceptions import UpkieException


class PlayTrajectory(gym.Wrapper):
    def __init__(self, env: UpkieServos, trajectory_path: Path):
        """Initialize environment.

        Args:
            env: UpkieServos environment to command servomotors.
        """
        super().__init__(env)
        if env.frequency is None:
            raise UpkieException("This environment needs a loop frequency")

        playback = env.get_neutral_action()

        df = pd.read_csv("my_csv.csv")
        df.set_index("t_grid", inplace=True)
        print(f"{df=}")

        self.df = df
        self.playback = playback
        self.playback_time = None

    def process_joystick(self, observation: dict):
        pass

    def step(self, action: dict) -> Tuple[np.ndarray, float, bool, bool, dict]:
        servo_action = action
        obs, reward, terminated, truncated, info = self.env.step(servo_action)
        self.process_joystick(info["spine_observation"])
        return obs, reward, terminated, truncated, info
