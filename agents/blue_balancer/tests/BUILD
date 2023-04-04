# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "kinematics_test",
    srcs = ["kinematics_test.py"],
    deps = [
        "//agents/blue_balancer:kinematics",
        requirement("numpy"),  # workaround
        # See https://github.com/tasts-robots/upkie_locomotion/issues/1
    ],
)

add_lint_tests()
