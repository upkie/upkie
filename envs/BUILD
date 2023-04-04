# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
        "standing_reward.py",
        "upkie_base_env.py",
        "upkie_servos_env.py",
        "upkie_wheels_env.py",
    ],
    data = [
        "spine.yaml",
    ],
    deps = [
        "@upkie_locomotion//observers/base_pitch",
        "@vulp//spine:python",
        requirement("gym"),
        requirement("pyyaml"),
        requirement("upkie_description"),
    ],
)

add_lint_tests()
