#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

from upkie.envs.pipelines import ServoPipeline
from upkie.envs.wrappers import Pendulum

from .upkie_mock_env import UpkieMockEnv
from .upkie_pybullet_env import UpkiePyBulletEnv
from .upkie_spine_env import UpkieSpineEnv


def wrap_pendulum(EnvClass, **kwargs):
    r"""!
    Make a new ground-velocity environment around an Upkie environment.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param kwargs Keyword arguments forwarded to both the
        \ref upkie.envs.wrappers.pendulum.Pendulum wrapper and the internal
        Upkie environment.
    \return New ground-velocity environment.
    """
    pendulum_keys = {
        "fall_pitch",
        "left_wheeled",
        "max_ground_velocity",
        "wheel_radius",
    }
    pendulum_kwargs = {
        key: value for key, value in kwargs.items() if key in pendulum_keys
    }
    servo_keys = {"max_gain_scale"}
    servo_kwargs = {
        key: value for key, value in kwargs.items() if key in servo_keys
    }
    env_kwargs = {
        key: value
        for key, value in kwargs.items()
        if key not in pendulum_keys and key not in servo_keys
    }
    servo_pipeline = ServoPipeline(**servo_kwargs)
    env = EnvClass(pipeline=servo_pipeline, **env_kwargs)
    return Pendulum(env, **pendulum_kwargs)


def wrap_mock_pendulum(**kwargs):
    return wrap_pendulum(UpkieMockEnv, **kwargs)


def wrap_spine_pendulum(**kwargs):
    return wrap_pendulum(UpkieSpineEnv, **kwargs)


def wrap_pybullet_pendulum(**kwargs):
    return wrap_pendulum(UpkiePyBulletEnv, **kwargs)
