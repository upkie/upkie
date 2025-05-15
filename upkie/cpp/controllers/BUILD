# -*- python -*-
#
# Copyright 2025 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controller",
    hdrs = ["Controller.h"],
    deps = [
        "@palimpsest",
    ],
)

cc_library(
    name = "controller_pipeline",
    hdrs = ["ControllerPipeline.h"],
    srcs = [
        "ControllerPipeline.cpp",
    ],
    deps = [
        "//upkie/cpp/exceptions",
        "//upkie/cpp/sensors:sensor",
        ":controller",
    ],
)

cc_library(
    name = "wheel_balancer",
    hdrs = ["WheelBalancer.h"],
    srcs = ["WheelBalancer.cpp"],
    deps = [
        ":controller",
    ],
)

cc_library(
    name = "wheel_stopper",
    hdrs = ["WheelStopper.h"],
    srcs = ["WheelStopper.cpp"],
    deps = [
        ":controller",
    ],
)

cc_library(
    name = "controllers",
    deps = [
        ":controller_pipeline",
    ],
)

add_lint_tests()
