# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "upkie",
    hdrs = [
        "upkie.h",
    ],
    deps = [
        "@vulp//actuation:servo_layout",
    ],
    include_prefix = "upkie_locomotion/spines",
)

add_lint_tests(enable_clang_format_lint = True)
