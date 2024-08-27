# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
#
# This file incorporates work covered by the following copyright and permission
# notice:
#
#     Copyright 2012-2016 Robot Locomotion Group @ CSAIL
#     SPDX-License-Identifier: BSD-3-Clause

load("//tools/lint:cpplint.bzl", "cpplint")

def add_lint_tests(
        cpplint_data = None,
        cpplint_extra_srcs = None,
        bazel_lint_ignore = None,
        bazel_lint_extra_srcs = None,
        bazel_lint_exclude = None,
        enable_clang_format_lint = True):
    """
    For every rule in the BUILD file so far, and for all Bazel files in this
    directory, adds test rules that run Drake's standard lint suite over the
    sources.  Thus, BUILD file authors should call this function at the *end*
    of every BUILD file.

    Refer to cpplint.bzl for semantics and argument details.

    """
    existing_rules = native.existing_rules().values()
    cpplint(
        existing_rules = existing_rules,
        data = cpplint_data,
        extra_srcs = cpplint_extra_srcs,
        enable_clang_format_lint = enable_clang_format_lint,
    )
