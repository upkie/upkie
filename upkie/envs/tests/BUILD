# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "mock_spine",
    srcs = ["mock_spine.py"],
)

py_test(
    name = "upkie_envs_test",
    srcs = ["upkie_envs_test.py"],
    deps = [
        "//upkie/envs",
        ":mock_spine",
    ],
)

py_test(
    name = "upkie_servos_test",
    srcs = ["upkie_servos_test.py"],
    deps = [
        "//upkie/envs",
        ":mock_spine",
    ],
)

py_test(
    name = "upkie_ground_velocity_test",
    srcs = ["upkie_ground_velocity_test.py"],
    deps = [
        "//upkie/envs",
        ":mock_spine",
    ],
)

add_lint_tests()
