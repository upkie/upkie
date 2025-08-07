# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "wrappers",
    srcs = [
        "__init__.py",
        "add_action_to_observation.py",
        "add_lag_to_action.py",
        "differentiate_action.py",
        "noisify_action.py",
        "noisify_observation.py",
        "pendulum.py",
        "random_push.py",
    ],
    deps = [
        "//upkie/envs:upkie_env",
        "//upkie/model",
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
        "//upkie:exceptions",
        "//upkie:logging",
    ],
)

add_lint_tests()
