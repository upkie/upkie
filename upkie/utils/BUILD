# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "datetime_now_string",
    hdrs = ["datetime_now_string.h"],
)

cc_library(
    name = "get_log_path",
    hdrs = ["get_log_path.h"],
    deps = [
        ":datetime_now_string",
    ],
    include_prefix = "vulp/utils",
)

py_library(
    name = "clamp",
    srcs = ["clamp.py"],
    deps = [
        ":spdlog",
    ],
)

py_library(
    name = "exceptions",
    srcs = ["exceptions.py"],
)

py_library(
    name = "filters",
    srcs = ["filters.py"],
    deps = [
        ":clamp",
    ],
)

py_library(
    name = "nested_update",
    srcs = ["nested_update.py"],
)

py_library(
    name = "pinocchio",
    srcs = ["pinocchio.py"],
)

py_library(
    name = "raspi",
    srcs = ["raspi.py"],
    deps = [
        ":spdlog",
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
    srcs = [
        "rotations.py",
    ],
)

py_library(
    name = "spdlog",
    srcs = ["spdlog.py"],
)

py_library(
    name = "utils",
    deps = [
        ":clamp",
        ":exceptions",
        ":filters",
        ":nested_update",
        ":pinocchio",
        ":raspi",
        ":spdlog",
        ":robot_state",
        ":rotations",
    ],
)

add_lint_tests()
