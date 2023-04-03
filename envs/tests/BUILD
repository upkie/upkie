# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "upkie_base_env_test",
    srcs = ["upkie_base_env_test.py"],
    deps = [
        "//envs",
        requirement("gin-config"),
        requirement("numpy"),  # workaround
        # See https://github.com/tasts-robots/upkie_locomotion/issues/1
    ],
)

py_test(
    name = "upkie_wheels_env_test",
    srcs = ["upkie_wheels_env_test.py"],
    deps = [
        "//envs",
        requirement("gin-config"),
        requirement("numpy"),  # workaround
        # See https://github.com/tasts-robots/upkie_locomotion/issues/1
    ],
)

add_lint_tests(enable_clang_format_lint = True)
