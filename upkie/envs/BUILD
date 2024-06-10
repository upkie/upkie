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
        "//upkie/utils:exceptions",
        "//upkie/utils:nested_update",
        "//upkie/utils:robot_state",
        "@vulp//:python",
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
        "//upkie/utils:robot_state",
    ],
)

py_library(
    name = "upkie_servos",
    srcs = [
        "upkie_servos.py",
    ],
    deps = [
        "//upkie/utils:pinocchio",
        "//upkie/utils:robot_state",
        ":upkie_base_env",
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
        ":upkie_servos",
    ],
)

add_lint_tests()
