# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "config",
    srcs = [
        "__init__.py",
    ],
    data = [
        "spine.yaml",
    ],
)

add_lint_tests()
