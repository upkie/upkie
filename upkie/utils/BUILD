# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "clamp",
    srcs = ["clamp.py"],
    deps = [
        "//upkie:logging",
    ],
)

py_library(
    name = "filters",
    srcs = ["filters.py"],
    deps = [
        ":clamp",
    ],
)

py_library(
    name = "joystick",
    srcs = ["joystick.py"],
    deps = [
    ],
)

py_library(
    name = "nested_update",
    srcs = ["nested_update.py"],
)

py_library(
    name = "raspi",
    srcs = ["raspi.py"],
    deps = [
        "//upkie:exceptions",
        "//upkie:logging",
    ],
)

py_library(
    name = "robot_state",
    srcs = [
        "robot_state.py",
        "robot_state_randomization.py",
    ],
)

py_library(
    name = "rotations",
    srcs = ["rotations.py"],
)

py_library(
    name = "utils",
    deps = [
        ":clamp",
        ":filters",
        ":joystick",
        ":nested_update",
        ":raspi",
        ":robot_state",
        ":rotations",
    ],
)

add_lint_tests()
