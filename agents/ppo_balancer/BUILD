# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_locomotion//:requirements.bzl", "requirement")

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
        requirement("gin-config"),
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
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
    ],
)

py_binary(
    name = "train",
    srcs = [
        "callbacks.py",
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
        "@rules_python//python/runfiles",
        requirement("gin-config"),
        requirement("stable_baselines3"),
    ],
)

add_lint_tests(enable_clang_format_lint = True)
