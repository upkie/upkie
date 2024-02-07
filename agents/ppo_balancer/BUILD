# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob(["*.gin"]),
)

py_library(
    name = "common",
    srcs = [
        "envs.py",
        "settings.py",
    ],
    data = [
        ":config",
    ],
    deps = [
        "//upkie/envs",
        "//upkie/envs/wrappers",
    ],
)

py_binary(
    name = "run",
    srcs = ["run.py"],
    main = "run.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        "policy/operative_config.gin",
        "policy/params.zip",
    ],
    deps = [
        "//upkie/envs",
        "//upkie/utils:filters",
        "//upkie/utils:raspi",
        "//upkie/utils:robot_state",
        ":common",
    ],
)

py_binary(
    name = "train",
    srcs = ["train.py"],
    main = "train.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        "//spines:bullet_spine",
    ],
    deps = [
        "//upkie/envs",
        "//upkie/utils:robot_state",
        "//upkie/utils:spdlog",
        ":common",
        "@rules_python//python/runfiles",
    ],
)

add_lint_tests()
