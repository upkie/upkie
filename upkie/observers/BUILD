# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_orientation",
    hdrs = ["BaseOrientation.h"],
    srcs = ["BaseOrientation.cpp"],
    deps = [
        "@vulp//vulp/observation:observer",
    ],
)

cc_library(
    name = "floor_contact",
    hdrs = ["FloorContact.h"],
    srcs = ["FloorContact.cpp"],
    deps = [
        "@vulp//vulp/observation:observer",
        ":wheel_contact",
    ],
)

cc_library(
    name = "wheel_contact",
    hdrs = ["WheelContact.h"],
    srcs = ["WheelContact.cpp"],
    deps = [
        "@vulp//vulp/observation:observer",
        "@vulp//vulp/utils:low_pass_filter",
    ],
)

cc_library(
    name = "wheel_odometry",
    hdrs = ["WheelOdometry.h"],
    srcs = ["WheelOdometry.cpp"],
    deps = [
        "@vulp//vulp/observation:observer",
    ],
)

cc_library(
    name = "observers",
    deps = [
        ":base_orientation",
        ":floor_contact",
        ":wheel_contact",
        ":wheel_odometry",
    ],
)

add_lint_tests()
