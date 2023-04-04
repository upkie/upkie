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
        "//utils:clamp",
    ],
)

add_lint_tests()
