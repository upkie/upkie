# -*- python -*-
#
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
    ],
)

py_binary(
    name = "model_predictive_control",
    srcs = ["model_predictive_control.py"],
    deps = [
        "//upkie/envs",
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
