# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "upkie_base_env_test",
    srcs = ["upkie_base_env_test.py"],
    deps = ["//envs"],
)

py_test(
    name = "upkie_servos_env_test",
    srcs = ["upkie_servos_env_test.py"],
    deps = ["//envs"],
)

py_test(
    name = "upkie_wheels_env_test",
    srcs = ["upkie_wheels_env_test.py"],
    deps = ["//envs"],
)

add_lint_tests()
