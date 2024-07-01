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
        "//vulp/actuation/moteus",
        "//vulp/observation:history_observer",
        "//vulp/observation",
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
