#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Environment wrapper to silence Upkie warnings in subprocess environments."""

import gymnasium as gym

from upkie.logging import disable_warnings


def silence_warnings(env: gym.Env) -> gym.Env:
    """
    Wrapper function to silence Upkie warnings in subprocess environments.

    This is called in each subprocess when using SubprocVecEnv, ensuring
    that warnings are suppressed in all parallel environments.

    Args:
        env: The environment to wrap

    Returns:
        The same environment (warnings are suppressed globally per process)
    """
    # Suppress warnings in this subprocess
    disable_warnings()
    return env
