# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "bullet_spine",
    srcs = [
        "bullet_spine.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/config:layout",
        "//upkie/observers",
        "//upkie/utils:datetime_now_string",
        "//upkie:version",
        "@vulp//vulp/actuation:bullet_interface",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
    ],
)

cc_binary(
    name = "mock_spine",
    srcs = [
        "mock_spine.cpp",
    ],
    deps = [
        "//upkie/config:layout",
        "//upkie/observers",
        "//upkie:version",
        "@vulp//vulp/actuation:mock_interface",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
    ] + select({
        "//:pi64_config": [
            "@org_llvm_libcxx//:libcxx",
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
        "//conditions:default": [],
    }),
)

cc_binary(
    name = "pi3hat_spine",
    srcs = [
        "pi3hat_spine.cpp",
    ],
    deps = [
        "//upkie/config:layout",
        "//upkie/observers",
        "//upkie/utils:datetime_now_string",
        "//upkie:version",
        "@vulp//vulp/observation",
        "@vulp//vulp/observation/sources",
        "@vulp//vulp/spine",
    ] + select({
        "//:pi64_config": ["@vulp//vulp/actuation:pi3hat_interface"],
        "//conditions:default": [],
    }),
    target_compatible_with = select({
        "//:pi64_config": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

add_lint_tests()
