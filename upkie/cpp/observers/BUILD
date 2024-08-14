# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_orientation",
    hdrs = ["BaseOrientation.h"],
    srcs = ["BaseOrientation.cpp"],
    deps = [
        ":observer",
    ],
)

cc_library(
    name = "floor_contact",
    hdrs = ["FloorContact.h"],
    srcs = ["FloorContact.cpp"],
    deps = [
        ":observer",
        ":wheel_contact",
    ],
)

cc_library(
    name = "history_observer",
    hdrs = ["HistoryObserver.h"],
    deps = [
        "//upkie/cpp/exceptions",
        ":observer",
    ],
)

cc_library(
    name = "observe_servos",
    hdrs = ["observe_servos.h"],
    srcs = ["observe_servos.cpp"],
    deps = [
        "//upkie/cpp/actuation/moteus",
        "@palimpsest",
    ],
)

cc_library(
    name = "observe_time",
    hdrs = ["observe_time.h"],
    deps = [
        "@palimpsest",
    ],
)

cc_library(
    name = "observer",
    hdrs = ["Observer.h"],
    deps = [
        "@palimpsest",
    ],
)

cc_library(
    name = "observer_pipeline",
    hdrs = ["ObserverPipeline.h"],
    srcs = [
        "ObserverPipeline.cpp",
    ],
    deps = [
        "//upkie/cpp/exceptions",
        "//upkie/cpp/sensors:sensor",
        ":observer",
    ],
)

cc_library(
    name = "wheel_contact",
    hdrs = ["WheelContact.h"],
    srcs = ["WheelContact.cpp"],
    deps = [
        ":observer",
        "//upkie/cpp/utils:low_pass_filter",
    ],
)

cc_library(
    name = "wheel_odometry",
    hdrs = ["WheelOdometry.h"],
    srcs = ["WheelOdometry.cpp"],
    deps = [
        ":observer",
    ],
)

cc_library(
    name = "observers",
    deps = [
        ":base_orientation",
        ":floor_contact",
        ":history_observer",
        ":observe_servos",
        ":observe_time",
        ":observer",
        ":observer_pipeline",
        ":wheel_contact",
        ":wheel_odometry",
    ],
)

add_lint_tests()
