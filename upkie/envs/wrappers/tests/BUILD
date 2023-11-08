# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "envs",
    srcs = ["envs.py"],
    deps = [
        requirement("gymnasium"),
    ],
)

py_test(
    name = "add_action_to_observation_test",
    srcs = ["add_action_to_observation_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
        requirement("gymnasium"),
        requirement("stable_baselines3"),
    ],
)

py_test(
    name = "differentiate_action_test",
    srcs = ["differentiate_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
        requirement("gymnasium"),
        requirement("stable_baselines3"),
    ],
)

py_test(
    name = "low_pass_filter_action_test",
    srcs = ["low_pass_filter_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
        requirement("gymnasium"),
        requirement("stable_baselines3"),
    ],
)

py_test(
    name = "noisify_action_test",
    srcs = ["noisify_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
        requirement("gymnasium"),
        requirement("stable_baselines3"),
    ],
)

py_test(
    name = "noisify_observation_test",
    srcs = ["noisify_observation_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
        requirement("gymnasium"),
        requirement("stable_baselines3"),
    ],
)

add_lint_tests()
