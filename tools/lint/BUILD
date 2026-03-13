# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

package(default_visibility = ["//visibility:public"])

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

py_binary(
    name = "clang_format_lint",
    srcs = ["clang_format_lint.py"],
    data = ["//:.clang-format"],
)

add_lint_tests()
