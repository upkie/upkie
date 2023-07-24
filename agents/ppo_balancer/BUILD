# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "settings",
    srcs = [
        "settings.py",
    ],
)

py_binary(
    name = "run",
    srcs = ["run.py"],
    data = glob([
        "policies/*.zip",
    ]),
    deps = [
        "//envs",
        ":settings",
        "@mpacklog//:python",
        requirement("loop_rate_limiters"),
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
        "@rules_python//python/runfiles",
        requirement("numpy"),
        requirement("stable_baselines3"),
        requirement("torch"),
    ],
)

add_lint_tests()
