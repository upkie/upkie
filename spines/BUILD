# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "airbullet",
    srcs = [
        "airbullet.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/config:layout",
        "//upkie/observers",
        "//upkie/utils:datetime_now_string",
        "@mpacklog",
        "@vulp//vulp/actuation:bullet_interface",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
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
        "//upkie/config:layout",
        "//upkie/observers",
        "//upkie/utils:datetime_now_string",
        "@mpacklog",
        "@vulp//vulp/actuation:bullet_interface",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
    ],
)

cc_binary(
    name = "mock",
    srcs = [
        "mock.cpp",
    ],
    deps = [
        "//upkie/config:layout",
        "//upkie/observers",
        "@vulp//vulp/actuation:mock_interface",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
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
        "//upkie/config:layout",
        "//upkie/observers",
        "//upkie/utils:datetime_now_string",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
    ] + select({
        "//:pi32_config": ["@vulp//vulp/actuation:pi3hat_interface"],
        "//:pi64_config": ["@vulp//vulp/actuation:pi3hat_interface"],
        "//conditions:default": [],
    }),
    target_compatible_with = select({
        "//:pi32_config": [],
        "//:pi64_config": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

add_lint_tests()
