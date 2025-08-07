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
    name = "upkie_genesis_env",
    srcs = [
        "upkie_genesis_env.py",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/config",
        "//upkie/envs/pipelines",
        "//upkie/model",
        "//upkie/utils",
        "//upkie:exceptions",
        ":upkie_env",
    ],
)

py_library(
    name = "upkie_mock_env",
    srcs = [
        "upkie_mock_env.py",
    ],
    deps = [
        "//upkie/envs/pipelines",
        "//upkie/utils",
        ":upkie_env",
    ],
)

py_library(
    name = "upkie_pybullet_env",
    srcs = [
        "upkie_pybullet_env.py",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/config",
        "//upkie/envs/pipelines",
        "//upkie/model",
        "//upkie/utils",
        "//upkie:exceptions",
        ":upkie_env",
    ],
)

py_library(
    name = "upkie_spine_env",
    srcs = [
        "upkie_spine_env.py",
    ],
    deps = [
        "//upkie/envs/pipelines",
        "//upkie/spine",
        "//upkie/utils",
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
        "//upkie/envs/pipelines",
        "//upkie/envs/wrappers",
        ":upkie_env",
        ":upkie_genesis_env",
        ":upkie_mock_env",
        ":upkie_pybullet_env",
        ":upkie_spine_env",
    ],
)

add_lint_tests()
