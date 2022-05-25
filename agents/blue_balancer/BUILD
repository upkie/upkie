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
    name = "wheel_balancer",
    srcs = [
        "wheel_balancer.py",
    ],
    deps = [
        "//utils:clamp",
        "//utils:filters",
        "//utils:imu",
        "@pip//pypi__gin_config",
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
    ],
    deps = [
        "//utils:realtime",
        ":whole_body_controller",
        "@pip//pypi__aiorate",
        "@pip//pypi__gin_config",
        "@pip//pypi__PyYAML",
        "@vulp//spine:python",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
