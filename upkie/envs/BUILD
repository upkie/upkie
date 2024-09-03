# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "upkie_base_env",
    srcs = ["upkie_base_env.py"],
    deps = [
        "//upkie/config",
        "//upkie/model",
        "//upkie/spine",
        "//upkie/utils:robot_state",
        "//upkie/utils:spdlog",
        "//upkie:exceptions",
    ],
)

py_library(
    name = "upkie_ground_velocity",
    srcs = [
        "upkie_ground_velocity.py",
    ],
    deps = [
        "//upkie/envs/rewards",
        "//upkie/utils:filters",
        ":upkie_base_env",
    ],
)

py_library(
    name = "upkie_servo_positions",
    srcs = [
        "upkie_servo_positions.py",
    ],
    deps = [
        ":upkie_servos",
    ],
)

py_library(
    name = "upkie_servo_torques",
    srcs = [
        "upkie_servo_torques.py",
    ],
    deps = [
        ":upkie_servos",
    ],
)

py_library(
    name = "upkie_servos",
    srcs = [
        "upkie_servos.py",
    ],
    deps = [
        "//upkie/utils:robot_state",
        ":upkie_base_env",
    ],
)

py_library(
    name = "wheeled_inverted_pendulum",
    srcs = [
        "wheeled_inverted_pendulum.py",
    ],
    deps = [
        "//upkie/envs/rewards",
        "//upkie/model",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
    ],
    deps = [
        ":upkie_base_env",
        ":upkie_ground_velocity",
        ":upkie_servo_positions",
        ":upkie_servo_torques",
        ":upkie_servos",
        ":wheeled_inverted_pendulum",
    ],
)

add_lint_tests()
