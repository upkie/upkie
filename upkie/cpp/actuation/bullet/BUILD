# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utils",
    hdrs = [
        "utils.h",
    ],
    deps = [
        "//upkie/cpp/actuation:bullet_imu_data",
        "@bullet",
        "@eigen",
        "@palimpsest",
    ],
)

add_lint_tests()
