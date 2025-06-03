# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "upkie_servos",
    srcs = [
        "upkie_servos.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/model",
        "//upkie/spine",
        "//upkie/utils:nested_update",
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
        "//upkie/utils:filters",
        ":upkie_servos",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
    ],
    deps = [
        ":upkie_ground_velocity",
        ":upkie_servos",
    ],
)

add_lint_tests()
