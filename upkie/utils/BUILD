# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "datetime_now_string",
    hdrs = ["datetime_now_string.h"],
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
    name = "spdlog",
    srcs = ["spdlog.py"],
)

py_library(
    name = "transforms",
    srcs = [
        "rigid_body_state.py",
        "rigid_body_state_randomization.py",
        "rotations.py",
    ],
)

py_library(
    name = "python",
    deps = [
        ":clamp",
        ":exceptions",
        ":filters",
        ":nested_update",
        ":pinocchio",
        ":raspi",
        ":spdlog",
        ":transforms",
    ],
)

add_lint_tests()
