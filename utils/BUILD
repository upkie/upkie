# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "clamp",
    srcs = [
        "clamp.py",
    ],
)

py_library(
    name = "filters",
    srcs = [
        "filters.py",
    ],
    deps = [
        ":clamp",
    ],
)

py_library(
    name = "imu",
    srcs = [
        "imu.py",
    ],
    deps = [
        ":clamp",
    ],
)

py_library(
    name = "realtime",
    srcs = [
        "realtime.py",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
