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

cc_binary(
    name = "mock",
    srcs = [
        "mock.cpp",
    ],
    deps = [
        "//observers",
        "//spines:upkie",
        "@vulp//actuation:mock_interface",
        "@vulp//observation",
        "@vulp//observation/sources",
        "@vulp//spine",
    ] + select({
        "//:pi32_config": ["@org_llvm_libcxx//:libcxx"],
        "//:pi64_config": ["@org_llvm_libcxx//:libcxx"],
        "//conditions:default": [],
    }),
)

add_lint_tests(enable_clang_format_lint = True)
