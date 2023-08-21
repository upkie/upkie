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
        requirement("gin-config"),
        requirement("ltv-mpc"),
        requirement("numpy"),
        requirement("qpsolvers"),
    ],
)

add_lint_tests()
