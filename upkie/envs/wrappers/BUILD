# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "wrappers",
    srcs = [
        "__init__.py",
        "add_action_to_observation.py",
        "differentiate_action.py",
        "low_pass_filter_action.py",
        "noisify_action.py",
        "noisify_observation.py",
    ],
    deps = [
        "//upkie/utils:exceptions",
        "//upkie/utils:filters",
    ],
)

add_lint_tests()
