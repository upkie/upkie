# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob([
        "*.gin",
        "*.yaml",
    ]),
)

py_library(
    name = "kinematics",
    srcs = [
        "kinematics.py",
    ],
    deps = [
        "//utils:clamp",
        "@pip_upkie_locomotion//pypi__gin_config",
    ],
)

py_library(
    name = "wheel_balancer",
    srcs = [
        "wheel_balancer.py",
    ],
    deps = [
        "//observers/base_pitch",
        "//utils:clamp",
        "//utils:filters",
        "@pip_upkie_locomotion//pypi__gin_config",
    ],
)

py_library(
    name = "whole_body_controller",
    srcs = [
        "whole_body_controller.py",
    ],
    deps = [
        ":kinematics",
        ":wheel_balancer",
        "@pip_upkie_locomotion//pypi__gin_config",
    ],
)

py_binary(
    name = "blue_balancer",
    srcs = [
        "main.py",
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
        "//utils:realtime",
        "//utils:spdlog",
        ":whole_body_controller",
        "@mpacklog//:python",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@pip_upkie_locomotion//pypi__loop_rate_limiters",
        "@pip_upkie_locomotion//pypi__pyyaml",
        "@vulp//spine:python",
    ],
)

py_binary(
    name = "bullet",
    srcs = [
        "bullet.py",
    ],
    main = "bullet.py",
    data = [
        ":config",
        "//spines:bullet",
    ],
    deps = [
        ":whole_body_controller",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@pip_upkie_locomotion//pypi__loop_rate_limiters",
        "@pip_upkie_locomotion//pypi__pyyaml",
        "@rules_python//python/runfiles",
        "@vulp//spine:python",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
