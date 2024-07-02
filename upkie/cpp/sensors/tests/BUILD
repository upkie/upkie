# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = select({
        "@//:linux": glob([
            "*.cpp",
            "*.h",
        ]),
        "@//conditions:default": glob([
            "*.cpp",
            "*.h",
        ], exclude=["JoystickTest.cpp"]),
    }),
    deps = [
        "//upkie/cpp/sensors",
        "@googletest//:main",
    ],
)

add_lint_tests()
