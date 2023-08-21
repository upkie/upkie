# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    deps = [
        "//upkie/utils:datetime_now_string",
        "@eigen",
        "@googletest//:main",
    ],
)

py_test(
    name = "clamp_test",
    srcs = ["clamp_test.py"],
    deps = [
        "//upkie/utils:clamp",
    ],
)

py_test(
    name = "pinocchio_test",
    srcs = ["pinocchio_test.py"],
    deps = [
        "//upkie/utils:pinocchio",
    ],
)

add_lint_tests()
