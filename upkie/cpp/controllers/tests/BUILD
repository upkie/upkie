# -*- python -*-
#
# Copyright 2025 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    deps = [
        "//upkie/cpp/controllers",
        "//upkie/cpp/exceptions",
        "@eigen",
        "@googletest//:main",
    ],
)

add_lint_tests()
