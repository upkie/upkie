# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob([
        "*.gin",
    ]),
)

py_library(
    name = "whole_body_controller",
    srcs = [
        "whole_body_controller.py",
    ],
    deps = [
        "//agents/blue_balancer:wheel_balancer",
        "@pip_upkie_locomotion//pypi__gin_config",
        # "@pip_upkie_locomotion//pypi__numpy",
        "@pip_upkie_locomotion//pypi__upkie_description",
    ],
)

py_binary(
    name = "pink_balancer",
    srcs = [
        "main.py",
    ],
    main = "main.py",
    data = [
        "//agents/blue_balancer:config",
        ":config",
    ],
    deps = [
        "//utils:realtime",
        "//utils:spdlog",
        ":whole_body_controller",
        "@mpacklog//python",
        "@pip_upkie_locomotion//pypi__aiorate",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@vulp//spine:python",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
