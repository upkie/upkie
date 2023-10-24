# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "pink_balancer",
    srcs = [
        "main.py",
        "wheel_balancer.py",
        "whole_body_controller.py",
    ],
    main = "main.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        ":config",
    ] + select({
        "//:pi64_config": ["//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/config",
        "//upkie/observers/base_pitch",
        "//upkie/utils:clamp",
        "//upkie/utils:exceptions",
        "//upkie/utils:filters",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
        "@vulp//:python",
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("packaging"),
        requirement("upkie_description"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
        ],
    }),
)

add_lint_tests()
