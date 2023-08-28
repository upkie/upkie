# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "rewards",
    srcs = [
        "reward.py",
        "survival_reward.py",
    ],
    deps = select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi32_config": [],
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
        ],
    })
)

py_library(
    name = "upkie_base_env",
    srcs = [
        "reward.py",
        "upkie_base_env.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/observers/base_pitch",
        ":rewards",
        "@vulp//:python",
        requirement("gymnasium"),
        requirement("loop_rate_limiters"),
        requirement("upkie_description"),
    ],
)

py_library(
    name = "upkie_servos_env",
    srcs = [
        "upkie_servos_env.py",
    ],
    deps = [
        ":upkie_base_env",
        "//upkie/utils:pinocchio",
    ],
)

py_library(
    name = "upkie_pendulum_env",
    srcs = [
        "upkie_pendulum_env.py",
    ],
    deps = [
        ":upkie_base_env",
    ],
)

py_library(
    name = "upkie_ground_accel_env",
    srcs = [
        "upkie_ground_accel_env.py",
    ],
    deps = [
        ":upkie_pendulum_env",
    ],
)

py_library(
    name = "upkie_ground_velocity_env",
    srcs = [
        "upkie_ground_velocity_env.py",
    ],
    deps = [
        ":upkie_pendulum_env",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
    ],
    deps = [
        ":rewards",
        ":upkie_base_env",
        ":upkie_servos_env",
        ":upkie_ground_velocity_env",
        requirement("gymnasium"),
    ],
)

add_lint_tests()
