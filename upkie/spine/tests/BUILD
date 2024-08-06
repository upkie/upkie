# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "spine_interface_test",
    srcs = [
        "spine_interface_test.py",
    ],
    deps = [
        "//upkie/spine",
    ],
)

add_lint_tests()
