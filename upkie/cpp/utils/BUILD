# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "handle_interrupts",
    hdrs = ["handle_interrupts.h"],
    srcs = [
        "handle_interrupts.cpp",
    ],
)

cc_library(
    name = "low_pass_filter",
    hdrs = ["low_pass_filter.h"],
    deps = [
        "//upkie/cpp/exceptions:filter_error",
        "@eigen",
    ],
)

cc_library(
    name = "math",
    hdrs = ["math.h"],
)

cc_library(
    name = "random_string",
    hdrs = ["random_string.h"],
)

cc_library(
    name = "realtime",
    hdrs = ["realtime.h"],
)

cc_library(
    name = "synchronous_clock",
    hdrs = ["SynchronousClock.h"],
    srcs = [
        "SynchronousClock.cpp",
    ],
    deps = [
        ":math",
        "@spdlog",
    ],
)

cc_library(
    name = "utils",
    deps = [
        ":handle_interrupts",
        ":low_pass_filter",
        ":math",
        ":random_string",
        ":realtime",
        ":synchronous_clock",
    ],
)

py_library(
    name = "serialize",
    srcs = ["serialize.py"],
)

py_library(
    name = "python",
    srcs = ["__init__.py"],
    deps = [
        ":serialize",
    ],
)

add_lint_tests()
