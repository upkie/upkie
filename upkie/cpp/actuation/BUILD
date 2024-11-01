# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "imu",
    hdrs = [
        "ImuData.h",
        "ImuUncertainty.h",
    ],
    deps = [
        "@eigen",
        "@palimpsest",
    ],
)

cc_library(
    name = "servo_layout",
    hdrs = [
        "ServoLayout.h",
    ],
    deps = [
        "//upkie/cpp/actuation/moteus",
        "//upkie/cpp/model",
    ],
)

cc_library(
    name = "static_config",
    hdrs = [
        "static_config.h",
    ],
    deps = [
        ":servo_layout",
    ],
)

cc_library(
    name = "interface",
    hdrs = [
        "default_action.h",
        "Interface.h",
    ],
    srcs = [
        "Interface.cpp",
    ],
    deps = [
        "//upkie/cpp/actuation/moteus",
        "//upkie/cpp/exceptions",
        "//upkie/cpp/model",
        ":imu",
        ":static_config",
        "@palimpsest",
    ],
)

cc_library(
    name = "mock_interface",
    hdrs = [
        "MockInterface.h",
    ],
    srcs = [
        "MockInterface.cpp",
    ],
    deps = [
        "//upkie/cpp/actuation:interface",
        "//upkie/cpp/utils:synchronous_clock",
        "@eigen",
    ],
)

cc_library(
    name = "bullet_interface",
    hdrs = [
        "BulletInterface.h",
    ],
    srcs = [
        "BulletInterface.cpp",
    ],
    data = [
        "//upkie/cpp/actuation/bullet/plane",
    ],
    deps = [
        "//upkie/cpp/actuation/bullet",
        "//upkie/cpp/actuation:interface",
        "//upkie/cpp/utils:synchronous_clock",
        "@bazel_tools//tools/cpp/runfiles",
        "@bullet",
        "@eigen",
        "@palimpsest",
    ],
)

cc_library(
    name = "pi3hat_interface",
    hdrs = [
        "Pi3HatInterface.h",
    ],
    srcs = select({
        "//:pi64_config": ["Pi3HatInterface.cpp"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/cpp/actuation/pi3hat:utils",
        "//upkie/cpp/utils:realtime",
        ":interface",
    ] + select({
        "//:pi64_config": [
            "@org_llvm_libcxx//:libcxx",
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "actuation",
    deps = [
        ":bullet_interface",
        ":mock_interface",
    ] + select({
        "//:pi64_config": [":pi3hat_interface"],
        "//conditions:default": [],
    }),
)

add_lint_tests()
