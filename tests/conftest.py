#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Pytest configuration and shared fixtures."""

import pytest

import upkie.envs


@pytest.fixture(scope="session", autouse=True)
def register_upkie_envs():
    """Register Upkie environments once per test session."""
    upkie.envs.register()
