# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "envs",
    srcs = ["envs.py"],
)

py_test(
    name = "add_action_to_observation_test",
    srcs = ["add_action_to_observation_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
    ],
)

py_test(
    name = "differentiate_action_test",
    srcs = ["differentiate_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
    ],
)

py_test(
    name = "low_pass_filter_action_test",
    srcs = ["low_pass_filter_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
    ],
)

py_test(
    name = "noisify_action_test",
    srcs = ["noisify_action_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
    ],
)

py_test(
    name = "noisify_observation_test",
    srcs = ["noisify_observation_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        ":envs",
    ],
)

py_test(
    name = "random_push_test",
    srcs = ["random_push_test.py"],
    deps = [
        "//upkie/envs/wrappers",
        "//upkie/envs",
        ":mock_spine",
        ":envs",
    ],
)

add_lint_tests()
