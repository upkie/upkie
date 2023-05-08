# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "servo_controller",
    srcs = [
        "servo_controller.py",
        "wheel_balancer.py",
    ],
    deps = [
        "//observers/base_pitch",
        "//utils:clamp",
        "//utils:filters",
        requirement("gin-config"),
    ],
)

py_binary(
    name = "agent_only",
    srcs = ["agent_only.py"],
    main = "agent_only.py",
    data = [
        "//agents/test_balancer/config",
    ] + select({
        "//:pi32_config": ["//spines:pi3hat"],
        "//:pi64_config": ["//spines:pi3hat"],
        "//conditions:default": [],
    }),
    deps = [
        "//utils:realtime",
        "//utils:spdlog",
        ":servo_controller",
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
        ":servo_controller",
        "@rules_python//python/runfiles",
        "@vulp//spine:python",
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
    ],
)

add_lint_tests()
