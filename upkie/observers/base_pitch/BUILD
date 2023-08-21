# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "base_pitch",
    srcs = [
        "__init__.py",
        "base_pitch.py",
    ],
    deps = [
        "//upkie/utils:clamp",
        "//upkie/utils:rotations",
    ],
)

add_lint_tests()
