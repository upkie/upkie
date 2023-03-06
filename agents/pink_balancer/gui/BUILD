# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "check_whole_body_controller",
    srcs = ["check_whole_body_controller.py"],
    deps = [
        "//agents/pink_balancer:whole_body_controller",
        requirement("numpy"),  # workaround
        # See https://github.com/tasts-robots/upkie_locomotion/issues/1
    ],
)

add_lint_tests(enable_clang_format_lint = True)
