# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "whole_body_controller_test",
    srcs = ["whole_body_controller_test.py"],
    deps = [
        "//agents/pink_balancer:whole_body_controller",
        "@pip_upkie_locomotion//pypi__numpy",  # workaround
        # See https://github.com/tasts-robots/upkie_locomotion/issues/1
    ],
)

add_lint_tests(enable_clang_format_lint = True)
