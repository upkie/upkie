# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "kinematics_test",
    srcs = ["kinematics_test.py"],
    deps = [
        "//agents/blue_balancer:kinematics",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
