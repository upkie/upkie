# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0
#
# This BUILD file allows launching the examples through Bazel. You don't need
# to go through Bazel if you have installed the "upkie" Python package from
# PyPI (pip install upkie). In the latter case, just run Python on example
# scripts directly.

load("//tools/lint:lint.bzl", "add_lint_tests")

py_binary(
    name = "lying_genuflection",
    srcs = ["lying_genuflection.py"],
    deps = [
        "//upkie/envs",
        "//upkie/observers/base_pitch",
    ],
)

py_binary(
    name = "mpc_balancing",
    srcs = ["mpc_balancing.py"],
    deps = [
        "//upkie/envs",
        "//upkie/observers/base_pitch",
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
    ],
)

py_binary(
    name = "wheeled_balancing",
    srcs = ["wheeled_balancing.py"],
    deps = [
        "//upkie/envs",
    ],
)

add_lint_tests()
