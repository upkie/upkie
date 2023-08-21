# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "base_pitch_test",
    srcs = ["base_pitch_test.py"],
    deps = [
        "//observers/base_pitch",
    ],
)

add_lint_tests()
