#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""This example simply makes Upkie go to its neutral configuration."""

from pathlib import Path
from typing import Dict

import gymnasium as gym
import pandas as pd

import upkie.envs

upkie.envs.register()


class SeriesStepper:
    def __init__(self, csv_path: Path):
        data = pd.read_csv(csv_path)
        timestamps = data.iloc[:, 0].values
        last = len(timestamps) - 1
        if not all(timestamps[i] <= timestamps[i + 1] for i in range(last)):
            raise ValueError("Timestamps in the CSV file should be be sorted.")

        self.cur_index = 0
        self.cur_time = timestamps[0]
        self.data = data
        self.timestamps = timestamps

    @property
    def terminated(self) -> bool:
        return self.cur_index >= len(self.timestamps) - 1

    def step(self, dt: float) -> Dict[str, float]:
        self.cur_time += dt
        last_index = len(self.timestamps) - 1
        while (
            self.cur_index < last_index
            and self.timestamps[self.cur_index] < self.cur_time
        ):
            self.cur_index += 1
        return self.data.iloc[self.cur_index].to_dict()


if __name__ == "__main__":
    trajectory = SeriesStepper("upkie_jump.csv")
    with gym.make("Upkie-Servos-Spine", frequency=200.0) as env:
        action = env.unwrapped.get_neutral_action()
        dt = env.unwrapped.dt
        observation, info = env.reset()  # connects to the spine
        data = trajectory.step(0)
        for _ in range(1000):
            action["left_hip"]["position"] = data["q_opt_7"]
            action["left_knee"]["position"] = data["q_opt_8"]
            action["left_wheel"]["position"] = data["q_opt_9"]
            action["right_hip"]["position"] = data["q_opt_10"]
            action["right_knee"]["position"] = data["q_opt_11"]
            action["right_wheel"]["position"] = data["q_opt_12"]

            action["left_hip"]["velocity"] = data["v_opt_6"]
            action["left_knee"]["velocity"] = data["v_opt_7"]
            action["left_wheel"]["velocity"] = data["v_opt_8"]
            action["right_hip"]["velocity"] = data["v_opt_9"]
            action["right_knee"]["velocity"] = data["v_opt_10"]
            action["right_wheel"]["velocity"] = data["v_opt_11"]

            data = trajectory.step(dt)
            if trajectory.terminated:
                break
            env.step(action)
