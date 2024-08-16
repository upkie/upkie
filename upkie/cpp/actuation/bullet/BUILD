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
        "JointProperties.h",
        "gravity.h",
        "read_imu_data.h",
        "utils.h",
    ],
    deps = [
        "//upkie/cpp/actuation:imu",
        "@bullet",
        "@eigen",
        "@palimpsest",
    ],
)

add_lint_tests()
