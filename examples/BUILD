# -*- python -*-
#
# Copyright 2022 Stéphane Caron
#
# This BUILD file allows launching the examples through Bazel. You don't need
# to go through Bazel if you have installed the "upkie" Python package from
# PyPI (pip install upkie). In the latter case, just run Python on example
# scripts directly.

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

py_binary(
    name = "wheeled_balancing",
    srcs = ["wheeled_balancing.py"],
    deps = [
        "//envs",
    ],
)

py_binary(
    name = "wheeled_balancing_servos_env",
    srcs = ["wheeled_balancing_servos_env.py"],
    deps = [
        "//envs",
        "//observers/base_pitch",
    ],
)

add_lint_tests()
