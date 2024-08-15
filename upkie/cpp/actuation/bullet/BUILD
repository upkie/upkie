# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bullet",
    hdrs = [
        "ContactData.h",
        "ExternalForce.h",
        "ImuData.h",
        "gravity.h",
        "imu.h",
        "utils.h",
    ],
    deps = [
        "//upkie/cpp/actuation:imu_data",
        "@bullet",
        "@eigen",
        "@palimpsest",
    ],
)

add_lint_tests()
