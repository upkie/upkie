# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "clamp_test",
    srcs = ["clamp_test.py"],
    deps = [
        "//upkie/utils:clamp",
    ],
)

py_test(
    name = "robot_state_test",
    srcs = ["robot_state_test.py"],
    deps = [
        "//upkie/utils:robot_state",
    ],
)

add_lint_tests()
