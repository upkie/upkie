#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""This example illustrates https://github.com/orgs/upkie/discussions/471"""

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np

import upkie.envs

upkie.envs.register()
NB_RUNS = 10
NB_STEPS = 100


if __name__ == "__main__":
    with gym.make("UpkieGroundVelocity-v3", frequency=200.0) as env:
        trajectories = []
        action = 0.0 * env.action_space.sample()
        for i in range(NB_RUNS):
            observation, _ = env.reset()  # connects to the spine
            trajectories.append([])
            for step in range(NB_STEPS):
                pitch = observation[0]
                ground_pos = observation[1]
                ground_vel = observation[3]
                action[0] = 10.0 * pitch + 1.0 * ground_pos + 0.1 * ground_vel
                observation, _, _, _, _ = env.step(action)
                trajectories[-1].append(observation[0])

        dt = env.unwrapped.dt
        trange = np.arange(0.0, NB_STEPS * dt, dt)
        plt.ion()
        plt.grid(True)
        plt.plot(trange, np.array(trajectories).T)
        plt.ylim(-0.03, 0.03)
        plt.legend(("pitch [rad]",))
        plt.show(block=True)
