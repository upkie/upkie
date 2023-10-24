# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob(["*.gin"]),
)

py_binary(
    name = "mpc_balancer",
    srcs = ["main.py"],
    main = "main.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        ":config",
    ] + select({
        "//:pi64_config": ["@upkie//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/envs",
        "//upkie/observers/base_pitch",
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
        requirement("gin-config"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
            requirement("qpmpc"),
            requirement("qpsolvers"),
        ],
    }),
)

add_lint_tests()
