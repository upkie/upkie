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
    data = [
        ":config",
    ] + select({
        "//:pi32_config": ["@upkie//spines:pi3hat"],
        "//:pi64_config": ["@upkie//spines:pi3hat"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/envs",
        "//upkie/observers/base_pitch",
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
        "//vulp/utils:serialize",
        requirement("gin-config"),
    ] + select({
        # Selecting the right CPU architecture for the Raspberry Pi is
        # currently a tough nut to crack for Bazel. We only enable compiled
        # dependencies on the host side for now.
        # Followed in: https://github.com/tasts-robots/upkie/issues/1
        "//:pi32_config": [],
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("ltv-mpc"),
            requirement("numpy"),
            requirement("qpsolvers"),
        ],
    }),
)

add_lint_tests()
