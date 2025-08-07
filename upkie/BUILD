# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "exceptions",
    srcs = ["exceptions.py"],
)

py_library(
    name = "logging",
    srcs = ["logging.py"],
)

py_library(
    name = "upkie",
    srcs = [
        "__init__.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/envs",
        "//upkie/spine",
        "//upkie/utils",
        ":exceptions",
        ":logging",
    ],
)

add_lint_tests()
