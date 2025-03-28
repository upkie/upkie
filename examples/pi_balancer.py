#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Pitch-position PI balancing controller."""

import gymnasium as gym
import upkie.envs
from upkie.utils.raspi import configure_agent_process, on_raspi

upkie.envs.register()


def main(
    env,
    pitch_kp: float = 10.0,
    pitch_ki: float = 1.0,
    position_kp: float = 10.0,
    position_ki: float = 1.0,
):
    pitch_integrator = 0.0
    position_integrator = 0.0
    dt = env.unwrapped.dt
    observation, _ = env.reset()  # connects to the spine
    action = 0.0 * env.action_space.sample()  # 1D action: [velocity]
    while True:
        pitch = observation[0]
        position = observation[1]
        pitch_integrator += pitch * dt
        position_integrator += position * dt
        commanded_velocity = (
            pitch_kp * pitch
            + pitch_ki * pitch_integrator
            + position_kp * position
            + position_ki * position_integrator
        )
        action[0] = commanded_velocity
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()
            pitch_integrator = 0.0
            position_integrator = 0.0


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()
    with gym.make("UpkieGroundVelocity-v4", frequency=100.0) as env:
        main(env)
