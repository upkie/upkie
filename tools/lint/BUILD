# -*- python -*-
#
# Copyright 2022 Stéphane Caron
#
# This file incorporates work covered by the following copyright and permission
# notice:
#
#     Copyright 2012-2016 Robot Locomotion Group @ CSAIL
#     License: BSD-3-Clause (see licenses/LICENSE-drake)

package(default_visibility = ["//visibility:public"])

load("//tools/lint:lint.bzl", "add_lint_tests")
load("//tools/lint:python_lint.bzl", "PYTHON_LINT_IGNORE_DEFAULT", "python_lint")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

py_binary(
    name = "clang_format_lint",
    srcs = ["clang_format_lint.py"],
    data = ["//:.clang-format"],
)

add_lint_tests()
