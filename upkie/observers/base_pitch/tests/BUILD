# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "base_pitch_test",
    srcs = ["base_pitch_test.py"],
    deps = [
        "//upkie/observers/base_pitch",
    ],
)

add_lint_tests()
