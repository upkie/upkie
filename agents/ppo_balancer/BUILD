# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "settings",
    srcs = [
        "settings.py",
    ],
    data = [
        "settings.gin",
    ],
    deps = [
        "@pip_upkie_locomotion//pypi__gin_config",
    ],
)

py_binary(
    name = "test",
    srcs = ["test.py"],
    data = glob([
        "policies/*.zip",
    ]),
    deps = [
        "//envs",
        ":settings",
        "@mpacklog//:python",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@pip_upkie_locomotion//pypi__loop_rate_limiters",
    ],
)

py_binary(
    name = "train",
    srcs = [
        "train.py",
    ],
    data = [
        "//spines:bullet",
    ],
    deps = [
        "//envs",
        "//utils:spdlog",
        ":settings",
        "@mpacklog//:python",
        "@pip_upkie_locomotion//pypi__gin_config",
        "@rules_python//python/runfiles",
    ],
)

add_lint_tests(enable_clang_format_lint = True)
