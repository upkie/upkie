# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "upkie_base_env",
    srcs = [
        "standing_reward.py",
        "upkie_base_env.py",
    ],
    deps = [
        "@upkie_locomotion//observers/base_pitch",
        "@vulp//spine:python",
        requirement("gym"),
        requirement("pyyaml"),
        requirement("upkie_description"),
    ],
)

py_library(
    name = "upkie_wheels_env",
    srcs = [
        "upkie_wheels_env.py",
    ],
    deps = [
        ":upkie_base_env",
    ],
)

py_library(
    name = "upkie_servos_env",
    srcs = [
        "upkie_servos_env.py",
    ],
    deps = [
        ":upkie_base_env",
        "//utils:pinocchio",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
    ],
    deps = [
        ":upkie_base_env",
        ":upkie_servos_env",
        ":upkie_wheels_env",
    ],
)

add_lint_tests()
