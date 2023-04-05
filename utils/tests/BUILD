# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "pinocchio_test",
    srcs = ["pinocchio_test.py"],
    deps = [
        "//utils:pinocchio",
    ],
)

add_lint_tests()
