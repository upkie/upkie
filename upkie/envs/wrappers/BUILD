# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "wrappers",
    srcs = [
        "__init__.py",
        "differentiate_action.py",
        "low_pass_filter_action.py",
        "noisify_action.py",
        "noisify_observation.py",
    ],
    deps = [
        "//upkie/utils:exceptions",
        "//upkie/utils:filters",
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("gymnasium"),
            requirement("numpy"),
        ],
    }),
)

add_lint_tests()
