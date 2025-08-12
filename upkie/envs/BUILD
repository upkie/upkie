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
        "//upkie/envs/backends",
        "//upkie/utils:robot_state",
    ],
)

py_library(
    name = "upkie_servos",
    srcs = [
        "upkie_servos.py",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/envs/backends",
        "//upkie/model",
        "//upkie/utils",
        "//upkie:exceptions",
        ":upkie_env",
    ],
)

py_library(
    name = "envs",
    srcs = [
        "__init__.py",
        "entry_points.py",
    ],
    deps = [
        "//upkie/envs/backends",
        "//upkie/envs/wrappers",
        ":upkie_env",
        ":upkie_servos",
    ],
)

add_lint_tests()