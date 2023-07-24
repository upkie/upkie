# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "pink_balancer",
    srcs = [
        "pink_balancer.py",
        "wheel_balancer.py",
        "whole_body_controller.py",
    ],
    main = "pink_balancer.py",
    data = [
        ":config",
    ] + select({
        "//:pi32_config": ["//spines:pi3hat"],
        "//:pi64_config": ["//spines:pi3hat"],
        "//conditions:default": [],
    }),
    deps = [
        "//config",
        "//observers/base_pitch",
        "//utils:clamp",
        "//utils:exceptions",
        "//utils:filters",
        "//utils:realtime",
        "//utils:spdlog",
        "@mpacklog//:python",
        "@vulp//spine:python",
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("packaging"),
        requirement("upkie_description"),
    ],
)

add_lint_tests()
