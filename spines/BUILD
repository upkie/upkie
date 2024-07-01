# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

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
        "//upkie/utils:get_log_path",
        "//upkie:version",
        "//upkie/cpp/actuation:bullet_interface",
        "//upkie/cpp/observation",
        "//upkie/cpp/observation/sources",
        "//upkie/cpp/observation:history_observer",
        "//upkie/cpp/spine",
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
        "//upkie/utils:get_log_path",
        "//upkie:version",
        "//upkie/cpp/actuation:mock_interface",
        "//upkie/cpp/observation",
        "//upkie/cpp/observation/sources",
        "//upkie/cpp/spine",
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
        "//upkie/utils:get_log_path",
        "//upkie:version",
        "//upkie/cpp/observation",
        "//upkie/cpp/observation/sources",
        "//upkie/cpp/spine",
    ] + select({
        "//:pi64_config": ["//upkie/cpp/actuation:pi3hat_interface"],
        "//conditions:default": [],
    }),
    target_compatible_with = select({
        "//:pi64_config": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

add_lint_tests()
