#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing with CPU isolation for improved performance on the raspi."""

import os
import sys

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()

CPUID = 3  # CPU core to use on the Raspberry Pi


def balance(env: gym.Env):
    """Run proportional balancer in gym environment."""
    observation = env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    for step in range(1_000_000):
        observation, reward, done, _ = env.step(action)
        if done:
            observation = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("Re-running as root so that we can set CPU affinity")
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    os.sched_setaffinity(0, {CPUID})
    env = gym.make("UpkieWheelsEnv-v4", frequency=200.0)
    try:
        balance(env)
    finally:  # make sure we disconnect from the spine
        env.close()
