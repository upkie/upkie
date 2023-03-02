# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "upkie_wheels_env_test",
    srcs = ["upkie_wheels_env_test.py"],
    deps = [
        "//envs",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@pip_upkie_locomotion//pypi__numpy",  # workaround
        # See https://github.com/tasts-robots/upkie_locomotion/issues/1
    ],
)

add_lint_tests(enable_clang_format_lint = True)
