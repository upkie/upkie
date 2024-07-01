# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

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
    name = "history_observer",
    hdrs = ["HistoryObserver.h"],
    deps = [
        "//upkie/cpp/exceptions",
        ":observer",
    ],
)

cc_library(
    name = "observer_pipeline",
    hdrs = ["ObserverPipeline.h"],
    srcs = [
        "ObserverPipeline.cpp",
    ],
    deps = [
        "//upkie/cpp/exceptions:observer_error",
        "//upkie/cpp/sensors:sensor",
        ":observer",
    ],
)

cc_library(
    name = "observers",
    deps = [
        ":observe_servos",
        ":observe_time",
        ":observer",
        ":observer_pipeline",
    ],
)

add_lint_tests()
