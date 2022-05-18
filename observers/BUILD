# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wheel_contact",
    hdrs = [
        "WheelContact.h",
    ],
    srcs = [
        "WheelContact.cpp",
    ],
    deps = [
        "@vulp//observation:observer",
        "@vulp//utils:low_pass_filter",
    ],
    include_prefix = "upkie_locomotion/observers",
)

cc_library(
    name = "floor_contact",
    hdrs = [
        "FloorContact.h",
      ],
    srcs = ["FloorContact.cpp"],
    deps = [
        "@vulp//observation:observer",
        ":wheel_contact",
    ],
    include_prefix = "upkie_locomotion/observers",
)

cc_library(
    name = "observers",
    deps = [
        ":floor_contact",
        ":wheel_contact",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
