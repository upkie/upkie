# -*- python -*-
#
# Copyright 2022 Stéphane Caron
#
# This file incorporates work covered by the following copyright and permission
# notice:
#
#     Copyright 2012-2016 Robot Locomotion Group @ CSAIL
#     License: BSD-3-Clause (see licenses/LICENSE-drake)

load("//tools/lint:cpplint.bzl", "cpplint")
load("//tools/lint:python_lint.bzl", "python_lint")

def add_lint_tests(
        cpplint_data = None,
        cpplint_extra_srcs = None,
        python_lint_ignore = ["E203"],
        python_lint_exclude = None,
        python_lint_extra_srcs = None,
        bazel_lint_ignore = None,
        bazel_lint_extra_srcs = None,
        bazel_lint_exclude = None,
        enable_clang_format_lint = True):
    """
    For every rule in the BUILD file so far, and for all Bazel files in this
    directory, adds test rules that run Drake's standard lint suite over the
    sources.  Thus, BUILD file authors should call this function at the *end*
    of every BUILD file.

    Refer to the specific linters for their semantics and argument details:
    - cpplint.bzl
    - python_lint.bzl

    We ignore E203 in Python linting by default as it is `not PEP 8 compliant
    <https://black.readthedocs.io/en/stable/the_black_code_style/current_style.html#slices>`_.
    """
    existing_rules = native.existing_rules().values()
    cpplint(
        existing_rules = existing_rules,
        data = cpplint_data,
        extra_srcs = cpplint_extra_srcs,
        enable_clang_format_lint = enable_clang_format_lint,
    )
    python_lint(
        existing_rules = existing_rules,
        ignore = python_lint_ignore,
        exclude = python_lint_exclude,
        extra_srcs = python_lint_extra_srcs,
    )
