# -*- python -*-
#
# Copyright 2022 Stéphane Caron

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
    data = [
        ":config",
    ] + select({
        "//:pi32_config": ["//spines:pi3hat"],
        "//:pi64_config": ["//spines:pi3hat"],
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
        "@mpacklog//:python",
        "@vulp//spine:python",
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("numpy"),
        requirement("packaging"),
        requirement("upkie_description"),
    ],
)

add_lint_tests()
