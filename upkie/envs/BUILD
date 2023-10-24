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
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
        ],
    })
)

py_library(
    name = "upkie_base_env",
    srcs = [
        "init_randomization.py",
        "upkie_base_env.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/observers/base_pitch",
        "//upkie/utils:exceptions",
        "//upkie/utils:nested_update",
        ":rewards",
        "@vulp//:python",
        requirement("loop-rate-limiters"),
        requirement("upkie_description"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("gymnasium"),
        ],
    }),
)

py_library(
    name = "upkie_servos",
    srcs = [
        "upkie_servos.py",
    ],
    deps = [
        ":upkie_base_env",
        "//upkie/utils:pinocchio",
    ],
)

py_library(
    name = "upkie_wheeled_pendulum",
    srcs = [
        "upkie_wheeled_pendulum.py",
    ],
    deps = [
        ":upkie_base_env",
    ],
)

py_library(
    name = "upkie_ground_velocity",
    srcs = [
        "upkie_ground_velocity.py",
    ],
    deps = [
        "//upkie/utils:exceptions",
        "//upkie/utils:filters",
        ":upkie_wheeled_pendulum",
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
        ":upkie_ground_velocity",
        ":upkie_servos",
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("gymnasium"),
        ],
    }),
)

add_lint_tests()
