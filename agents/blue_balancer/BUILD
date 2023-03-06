# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "kinematics",
    srcs = [
        "kinematics.py",
    ],
    deps = [
        "//utils:clamp",
        requirement("gin-config"),
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
        requirement("gin-config"),
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
        requirement("gin-config"),
    ],
)

py_binary(
    name = "blue_balancer",
    srcs = [
        "blue_balancer.py",
    ],
    main = "blue_balancer.py",
    data = [
        "//agents/blue_balancer/config",
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
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
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
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
        "@rules_python//python/runfiles",
        "@vulp//spine:python",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
