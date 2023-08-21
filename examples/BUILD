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
        # TODO(scaron): add ltv-mpc
        requirement("qpsolvers"),
    ],
)

py_binary(
    name = "wheeled_balancing",
    srcs = ["wheeled_balancing.py"],
    deps = [
        "//upkie/envs",
    ],
)

py_binary(
    name = "wheeled_balancing_async",
    srcs = ["wheeled_balancing_async.py"],
    deps = [
        "//upkie/envs",
    ],
)

add_lint_tests()
