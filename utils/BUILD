# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "clamp",
    srcs = ["clamp.py"],
    deps = [
        ":spdlog",
    ],
)

py_library(
    name = "filters",
    srcs = ["filters.py"],
    deps = [
        ":clamp",
    ],
)

py_library(
    name = "logging",
    srcs = ["logging.py"],
    deps = [
        "@vulp//utils:serialize",
        requirement("aiofiles"),
        requirement("msgpack"),
    ],
)

py_library(
    name = "pinocchio",
    srcs = ["pinocchio.py"],
)

py_library(
    name = "realtime",
    srcs = ["realtime.py"],
)

py_library(
    name = "spdlog",
    srcs = ["spdlog.py"],
)

add_lint_tests()
