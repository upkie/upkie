# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

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
        requirement("gin-config"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/upkie/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
        ],
    }),
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
        ":common",
        requirement("loop-rate-limiters"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/upkie/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("gymnasium"),
            requirement("stable_baselines3"),
        ],
    }),
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
        "//upkie/utils:spdlog",
        ":common",
        "@rules_python//python/runfiles",
        requirement("gin-config"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/upkie/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("gymnasium"),
            requirement("stable_baselines3"),
            requirement("torch"),
        ],
    }),
)

add_lint_tests()
