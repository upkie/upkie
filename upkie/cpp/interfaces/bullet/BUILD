# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bullet",
    hdrs = [
        "constants.h",
        "read_imu_data.h",
        "utils.h",
    ],
    deps = [
        "//upkie/cpp/interfaces:imu",
        "@bullet",
        "@eigen",
        "@palimpsest",
    ],
)

add_lint_tests()
