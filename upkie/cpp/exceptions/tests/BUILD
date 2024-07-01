# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    deps = [
        "//vulp/exceptions",
        "@eigen",
        "@googletest//:main",
    ],
)

add_lint_tests()
