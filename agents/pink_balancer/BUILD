# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob([
        "*.gin",
    ]),
)

py_library(
    name = "wheel_balancer",
    srcs = [
        "wheel_balancer.py",
    ],
    deps = [
        "//utils:clamp",
        "//utils:filters",
        "//utils:imu",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
