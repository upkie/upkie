# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "default_action",
    hdrs = [
        "default_action.h",
    ],
)

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
        "ServoProperties.h",
    ],
    deps = [
        "//upkie/cpp/interfaces/moteus",
        "//upkie/cpp/exceptions",
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
        "Interface.h",
    ],
    srcs = [
        "Interface.cpp",
    ],
    deps = [
        "//upkie/cpp/interfaces/moteus",
        "//upkie/cpp/exceptions",
        ":default_action",
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
        "//upkie/cpp/interfaces:interface",
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
        "//upkie/cpp/interfaces/bullet/plane",
    ],
    deps = [
        "//upkie/cpp/interfaces/bullet",
        "//upkie/cpp/interfaces:interface",
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
        "//upkie/cpp/interfaces/pi3hat:utils",
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
    name = "interfaces",
    deps = [
        ":bullet_interface",
        ":mock_interface",
    ] + select({
        "//:pi64_config": [":pi3hat_interface"],
        "//conditions:default": [],
    }),
)

add_lint_tests()
