# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "reward",
    srcs = [
        "reward.py",
    ],
)

py_library(
    name = "upkie_wheels_env",
    srcs = [
        "upkie_wheels_env.py",
    ],
    data = [
        "spine.yaml",
        "upkie_wheels_env.gin",
    ],
    deps = [
        "@upkie_locomotion//observers/base_pitch",
        "@vulp//spine:python",
        ":reward",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
    ],
    deps = [
        ":upkie_wheels_env",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
