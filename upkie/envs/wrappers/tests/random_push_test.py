#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test RandomPush wrapper."""

import unittest

import gymnasium
import numpy as np

from upkie.envs.wrappers.random_push import (
    RandomPush,
)
from upkie.envs import UpkieGroundVelocity


class RandomPushTestCase(unittest.TestCase):
    def test_wrapper(self):
        env = UpkieGroundVelocity()
        wrapped_env = RandomPush(
            env, 
            push_prob=1, 
            push_generator=lambda: np.array([42, 42, 42])
            )
        action = np.array([1.0])
        spine_action = wrapped_env.get_spine_action(action)
        assert "bullet" in spine_action
        assert "external_forces" in spine_action["bullet"]
        assert "torso" in spine_action["bullet"]["external_forces"]
        assert "force" in spine_action["bullet"]["external_forces"]["torso"]
        assert np.allclose(
            spine_action["bullet"]["external_forces"]["torso"]["force"], 
            np.array([42, 42, 42])
            )


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
