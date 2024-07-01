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
        "//vulp/actuation/moteus",
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
    name = "source",
    hdrs = ["Source.h"],
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
        "//upkie/cpp/exceptions:observer_error",
        ":observer",
        ":source",
    ],
)

cc_library(
    name = "observation",
    deps = [
        ":observe_servos",
        ":observe_time",
        ":observer",
        ":observer_pipeline",
        ":source",
    ],
)

add_lint_tests()
