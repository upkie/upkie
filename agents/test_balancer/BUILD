# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "wheel_balancer",
    srcs = [
        "controller.py",
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
        ":wheel_balancer",
        "@mpacklog//:python",
        "@vulp//spine:python",
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
    ],
)

py_binary(
    name = "standalone",
    srcs = ["standalone.py"],
    main = "standalone.py",
    data = [
        ":config",
        "//spines:bullet",
    ],
    deps = [
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
        "@rules_python//python/runfiles",
        "@vulp//spine:python",
    ],
)

add_lint_tests()
