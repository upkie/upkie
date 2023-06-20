# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "pink_balancer_test",
    srcs = ["pink_balancer_test.py"],
    deps = [
        "//agents/pink_balancer:pink_balancer",
    ],
)

py_test(
    name = "whole_body_controller_test",
    srcs = ["whole_body_controller_test.py"],
    data = [
        "//agents/pink_balancer/config",
    ],
    deps = [
        "//agents/pink_balancer:pink_balancer",
        requirement("gin-config"),
    ],
)

add_lint_tests()
