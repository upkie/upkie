# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "upkie_env",
    srcs = [
        "upkie_env.py",
    ],
    deps = [
        "//upkie/utils:robot_state",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
        "entry_points.py",
        "upkie_mock_env.py",
        "upkie_spine_env.py",
    ],
    deps = [
        ":upkie_env",
        "//upkie/envs/pipelines",
        "//upkie/envs/wrappers",
    ],
)

add_lint_tests()
