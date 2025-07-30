# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "pipelines",
    srcs = [
        "__init__.py",
        "pipeline.py",
        "servo_pipeline.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/model",
        "//upkie/utils:clamp",  
        "//upkie/utils:nested_update",
        "//upkie/utils:spdlog",
        "//upkie:exceptions",
    ],
)

add_lint_tests()