# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "tests",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    deps = [
        "//upkie/cpp/interfaces:mock_interface",
        "//upkie/cpp/observers/tests:observers",
        "//upkie/cpp/observers:observer_pipeline",
        "//upkie/cpp/spine:agent_interface",
        "//upkie/cpp/spine:spine",
        "//upkie/cpp/spine:state_machine",
        "//upkie/cpp/utils:random_string",
        "@googletest//:main",
        "@palimpsest",
    ],
)

add_lint_tests()
