# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

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
        "//observers/base_pitch",
        "//utils:clamp",
        "//utils:filters",
        "//utils:realtime",
        "//utils:spdlog",
        "@mpacklog//:python",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@pip_upkie_locomotion//pypi__loop_rate_limiters",
        "@pip_upkie_locomotion//pypi__upkie_description",
        "@vulp//spine:python",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
