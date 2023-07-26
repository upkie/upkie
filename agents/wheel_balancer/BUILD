# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "controllers",
    srcs = [
        "servo_controller.py",
        "wheel_controller.py",
    ],
    deps = [
        "//observers/base_pitch",
        "//utils:clamp",
        "//utils:filters",
        requirement("gin-config"),
    ],
)

py_binary(
    name = "agent",
    srcs = ["agent.py"],
    main = "agent.py",
    data = [
        "//agents/wheel_balancer/config",
    ] + select({
        "//:pi32_config": ["//spines:pi3hat"],
        "//:pi64_config": ["//spines:pi3hat"],
        "//conditions:default": [],
    }),
    deps = [
        "//utils:exceptions",
        "//utils:raspi",
        "//utils:spdlog",
        ":controllers",
        "@mpacklog//:python",
        "@vulp//spine:python",
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
    ],
)

py_binary(
    name = "bullet",
    srcs = ["bullet.py"],
    main = "bullet.py",
    data = [
        ":config",
        "//spines:bullet",
    ],
    deps = [
        "//utils:exceptions",
        ":controllers",
        "@rules_python//python/runfiles",
        "@vulp//spine:python",
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
    ],
)

add_lint_tests()
