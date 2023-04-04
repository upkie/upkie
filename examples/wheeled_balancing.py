#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

import gym

import upkie_locomotion.envs

upkie_locomotion.envs.register()

if __name__ == "__main__":
    env = gym.make("UpkieWheelsEnv-v1")
    observation = env.reset(seed=42)

    action = env.action_space.sample()
    for step in range(1_000_000):
        observation, reward, done, _ = env.step(action)
        if done:
            observation = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch

    env.close()
