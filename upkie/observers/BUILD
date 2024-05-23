# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(  # will end up in vulp but we develop it here for now
    name = "history_observer",
    hdrs = ["HistoryObserver.h"],
    srcs = ["HistoryObserver.cpp"],
    deps = [
        "@vulp//vulp/exceptions:observer_error",
        "@vulp//vulp/observation:observer",
    ],
    include_prefix = "vulp/observation",
)

cc_library(
    name = "floor_contact",
    hdrs = [
        "FloorContact.h",
      ],
    srcs = [
        "FloorContact.cpp",
    ],
    deps = [
        "@vulp//vulp/observation:observer",
        ":wheel_contact",
    ],
)

cc_library(
    name = "wheel_odometry",
    hdrs = [
        "WheelOdometry.h",
    ],
    srcs = [
        "WheelOdometry.cpp",
    ],
    deps = [
        "@vulp//vulp/observation:observer",
    ],
)

cc_library(
    name = "observers",
    deps = [
        ":floor_contact",
        ":wheel_contact",
        ":wheel_odometry",
    ],
)

add_lint_tests()
