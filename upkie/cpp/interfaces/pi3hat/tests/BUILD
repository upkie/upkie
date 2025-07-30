# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = [
      "imu_test.cpp",
    ],
    deps = [
        "//upkie/cpp/interfaces/pi3hat:utils",
        "@bazel_tools//tools/cpp/runfiles",
        "@googletest//:main",
    ],
)

add_lint_tests()
