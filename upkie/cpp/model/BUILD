# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model",
    hdrs = [
        "joints.h",
        "servo_layout.h",
    ],
    deps = [
        "//upkie/cpp/actuation:servo_layout",
    ],
)

add_lint_tests()
