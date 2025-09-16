# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bullet",
    hdrs = [
        "ContactData.h",
        "gravity.h",
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
