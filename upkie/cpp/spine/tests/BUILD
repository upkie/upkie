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
        "//vulp/actuation:mock_interface",
        "//vulp/observation/tests:observers",
        "//vulp/observation:observer_pipeline",
        "//vulp/spine:agent_interface",
        "//vulp/spine:spine",
        "//vulp/spine:state_machine",
        "//vulp/utils:random_string",
        "@googletest//:main",
        "@palimpsest",
    ],
)

py_test(
    name = "spine_interface_test",
    srcs = [
        "spine_interface_test.py",
    ],
    deps = [
        "//vulp:python",
    ],
)

add_lint_tests()
