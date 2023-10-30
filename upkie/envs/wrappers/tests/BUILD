# -*- python -*-
#
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "noisify_action_test",
    srcs = ["noisify_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        requirement("gymnasium"),
    ],
)

py_test(
    name = "noisify_observation_test",
    srcs = ["noisify_observation_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        requirement("gymnasium"),
    ],
)

add_lint_tests()
