#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Play a precomputed joint trajectory while balancing with the wheels."""

from copy import deepcopy
from pathlib import Path
from typing import Dict

import gymnasium as gym
import numpy as np
import pandas as pd

import upkie.envs
from upkie.controllers import MPCBalancer

upkie.envs.register()


class SeriesStepper:
    def __init__(self, csv_path: Path, max_time: float = 1e20):
        data = pd.read_csv(csv_path)
        timestamps = data.iloc[:, 0].values
        last = len(timestamps) - 1
        if not all(timestamps[i] <= timestamps[i + 1] for i in range(last)):
            raise ValueError("Timestamps in the CSV file should be be sorted.")
        max_time = (
            timestamps[last]
            if max_time is None
            else min(max_time, timestamps[last])
        )

        self.cur_index = 0
        self.cur_time = timestamps[0]
        self.max_time = max_time
        self.data = data
        self.timestamps = timestamps

    @property
    def terminated(self) -> bool:
        if self.cur_time >= self.max_time:
            return True
        return self.cur_index >= len(self.timestamps) - 1

    def reset(self) -> Dict[str, float]:
        self.cur_index = 0
        self.cur_time = self.timestamps[0]
        return self.step(0)

    def step(self, dt: float) -> Dict[str, float]:
        if self.cur_time >= self.max_time:
            return self.data.iloc[self.cur_index].to_dict()
        self.cur_time += dt
        last_index = len(self.timestamps) - 1
        print_message = False
        while (
            self.cur_index < last_index
            and self.timestamps[self.cur_index] < self.cur_time
        ):
            self.cur_index += 1
            if self.cur_index % 100 == 0:
                print_message = True
        if print_message:
            print(f"Trajectory playback is now at t = {self.cur_time:.1} s")
        return self.data.iloc[self.cur_index].to_dict()


class TrajectoryPlayer:

    def __init__(
        self,
        neutral_action,
        dt: float,
        trajectory,
        reset_duration: float,
        timescale: float,
    ):
        assert 0 < timescale < 1.05
        initial_action = neutral_action.copy()
        initial_action["left_wheel"]["position"] = np.nan
        initial_action["right_wheel"]["position"] = np.nan
        initial_action["left_wheel"]["velocity"] = 0.0
        initial_action["right_wheel"]["velocity"] = 0.0
        upper_leg_joints = [
            f"{side}_{name}"
            for side in ("left", "right")
            for name in ("hip", "knee")
        ]
        self.__action = initial_action
        self.data = None
        self.dt = dt
        self.playback = False
        self.rem_reset_steps = 0
        self.reset_duration = reset_duration
        self.timescale = timescale
        self.trajectory = trajectory
        self.upper_leg_joints = upper_leg_joints

    def reset(self, init_state: dict):
        for joint in self.upper_leg_joints:
            position = init_state[joint]["position"]
            self.__action[joint]["position"] = position
            self.__action[joint]["velocity"] = -position / self.reset_duration
        self.rem_reset_steps = int(self.reset_duration / self.dt)
        self.data = None
        self.playback = False

    def step_reset(self):
        for joint in self.upper_leg_joints:
            if self.rem_reset_steps > 0:
                action_vel = self.__action[joint]["velocity"]
                self.__action[joint]["position"] += action_vel * self.dt
            else:  # self.rem_reset_steps <= 0
                self.__action[joint]["position"] = 0.0
                self.__action[joint]["velocity"] = 0.0
        self.rem_reset_steps -= 1

    def step_trajectory(self):
        self.__action["left_hip"]["position"] = self.data["q_opt_7"]
        self.__action["left_knee"]["position"] = self.data["q_opt_8"]
        self.__action["right_hip"]["position"] = self.data["q_opt_10"]
        self.__action["right_knee"]["position"] = self.data["q_opt_11"]

        s_dot = self.timescale
        self.__action["left_hip"]["velocity"] = s_dot * self.data["v_opt_6"]
        self.__action["left_knee"]["velocity"] = s_dot * self.data["v_opt_7"]
        self.__action["right_hip"]["velocity"] = s_dot * self.data["v_opt_9"]
        self.__action["right_knee"]["velocity"] = s_dot * self.data["v_opt_10"]

        ds = s_dot * self.dt
        self.data = self.trajectory.step(ds)
        if self.trajectory.terminated:
            self.reset(init_state=self.__action)

    def check_joystick(self, spine_observation: dict) -> None:
        if "joystick" not in spine_observation:
            return

        joystick = spine_observation["joystick"]
        if joystick.get("cross_button", False):
            self.reset(self.__action)

        ready_to_play = not self.playback and self.rem_reset_steps < 1
        if joystick.get("square_button", False) and ready_to_play:
            print("Starting trajectory playback...")
            self.data = self.trajectory.reset()
            self.playback = True

    def step(self, spine_observation: dict) -> dict:
        self.check_joystick(spine_observation)
        if self.rem_reset_steps >= 0:
            self.step_reset()
        elif self.playback:
            self.step_trajectory()
        return deepcopy(self.__action)


if __name__ == "__main__":
    mpc_balancer = MPCBalancer(fall_pitch=1.5)
    trajectory = SeriesStepper("upkie_jump_v2_vel_limits.csv", max_time=0.5)
    with gym.make("Upkie-Servos-Spine", frequency=200.0) as env:
        observation, info = env.reset()  # connects to the spine
        player = TrajectoryPlayer(
            neutral_action=env.unwrapped.get_neutral_action(),
            dt=env.unwrapped.dt,
            trajectory=trajectory,
            timescale=0.1,
            reset_duration=3.0,
        )
        player.reset(observation)
        while True:
            action = player.step(spine_observation=info["spine_observation"])
            ground_velocity = mpc_balancer.compute_ground_velocity(
                target_ground_velocity=0.0,  # m/s
                spine_observation=info["spine_observation"],
                dt=env.unwrapped.dt,
            )
            WHEEL_RADIUS = 0.06  # meters
            wheel_velocity = ground_velocity / WHEEL_RADIUS
            action["left_wheel"]["velocity"] += wheel_velocity
            action["right_wheel"]["velocity"] -= wheel_velocity
            _, _, _, _, info = env.step(action)
