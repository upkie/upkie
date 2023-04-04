# -*- python -*-
#
# Copyright 2022 Stéphane Caron
#
# This BUILD file allows launching the examples through Bazel. You don't need
# to go through Bazel if you have installed upkie_locomotion from PyPI (pip
# install upkie_locomotion). In the latter case, just run Python on example
# scripts directly.

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

py_binary(
    name = "legged_balancing",
    srcs = ["legged_balancing.py"],
    deps = ["//envs"],
)

py_binary(
    name = "wheeled_balancing",
    srcs = ["wheeled_balancing.py"],
    deps = ["//envs"],
)

add_lint_tests(enable_clang_format_lint = True)
