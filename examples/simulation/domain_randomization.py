#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Apply some domain-randomization wrappers to an Upkie environment."""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.envs.wrappers import (
    AddLagToAction,
    NoisifyAction,
    NoisifyObservation,
    RandomPush,
)

upkie.envs.register()


def add_domain_randomization(
    env: gym.Env,
    action_noise: float = 0.05,  # [m] / [s]
    observation_noise: float = 0.01,  # mixed units (*^_^*)
) -> gym.Wrapper:
    obs_ones = np.ones(env.observation_space.shape)
    act_ones = np.ones(env.action_space.shape)
    env = RandomPush(
        env,
        push_prob=1.0 / 200,
        push_generator=lambda: np.random.uniform(-50, 50, 3),
    )
    env = NoisifyObservation(env, noise=observation_noise * obs_ones)
    env = NoisifyAction(env, noise=action_noise * act_ones)
    env = AddLagToAction(env, time_constant=0.15)
    return env


if __name__ == "__main__":
    with gym.make("UpkieGroundVelocity", frequency=200.0) as spine_env:
        env = add_domain_randomization(spine_env)
        observation, _ = env.reset()
        gain = np.array([10.0, 1.0, 0.0, 0.1])
        for step in range(1_000_000):
            action = gain.dot(observation).reshape((1,))
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
