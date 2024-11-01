# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model",
    hdrs = [
        "Joint.h",
        "joints.h",
    ],
    deps = [
        "//upkie/cpp/exceptions",
    ],
)

add_lint_tests()
