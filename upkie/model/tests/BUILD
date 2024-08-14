# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "model_test",
    srcs = ["model_test.py"],
    deps = [
        "//upkie/model",
    ],
)

add_lint_tests()
