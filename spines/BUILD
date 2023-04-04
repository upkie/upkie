# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "upkie_layout",
    hdrs = [
        "upkie_layout.h",
    ],
    deps = [
        "@vulp//actuation:servo_layout",
    ],
    include_prefix = "upkie_locomotion/spines",
)

cc_binary(
    name = "airbullet",
    srcs = [
        "airbullet.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//observers",
        ":upkie_layout",
        "@mpacklog",
        "@vulp//actuation:bullet_interface",
        "@vulp//observation",
        "@vulp//observation/sources",
        "@vulp//spine",
        "@vulp//utils:datetime_now_string",
    ],
)

cc_binary(
    name = "bullet",
    srcs = [
        "bullet.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//observers",
        ":upkie_layout",
        "@mpacklog",
        "@vulp//actuation:bullet_interface",
        "@vulp//observation",
        "@vulp//observation/sources",
        "@vulp//spine",
        "@vulp//utils:datetime_now_string",
    ],
)

cc_binary(
    name = "mock",
    srcs = [
        "mock.cpp",
    ],
    deps = [
        "//observers",
        ":upkie_layout",
        "@vulp//actuation:mock_interface",
        "@vulp//observation",
        "@vulp//observation/sources",
        "@vulp//spine",
    ] + select({
        "//:pi32_config": [
            "@org_llvm_libcxx//:libcxx",
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
        "//:pi64_config": [
            "@org_llvm_libcxx//:libcxx",
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
        "//conditions:default": [],
    }),
)

cc_binary(
    name = "pi3hat",
    srcs = ["pi3hat.cpp"],
    deps = [
        "//observers",
        ":upkie_layout",
        "@vulp//observation",
        "@vulp//observation/sources",
        "@vulp//spine",
        "@vulp//utils:datetime_now_string",
    ] + select({
        "//:pi32_config": ["@vulp//actuation:pi3hat_interface"],
        "//:pi64_config": ["@vulp//actuation:pi3hat_interface"],
        "//conditions:default": [],
    }),
    target_compatible_with = select({
        "//:pi32_config": [],
        "//:pi64_config": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

add_lint_tests()
