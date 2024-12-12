# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    deps = [
        "//upkie/cpp/actuation/moteus",
        "//upkie/cpp/exceptions",
        "//upkie/cpp/observers",
        "@eigen",
        "@googletest//:main",
    ],
)

cc_library(
    name = "observers",
    hdrs = [
        "SchwiftyObserver.h",
        "ThrowingObserver.h",
    ],
)

add_lint_tests()
