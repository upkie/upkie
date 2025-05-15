# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    hdrs = [
        "observers.h",
        "sensors.h",
    ],
    deps = [
        "//upkie/cpp/exceptions",
        "//upkie/cpp/observers",
        "//upkie/cpp/sensors",
    ]
)

add_lint_tests()
