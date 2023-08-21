# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob(["*.gin"]),
)

py_library(
    name = "rewards",
    srcs = [
        "standing_reward.py",
    ],
    deps = [
        "//envs",
        requirement("numpy"),
    ],
)

py_library(
    name = "settings",
    srcs = ["settings.py"],
    data = [
        ":config",
    ],
    deps = [
        requirement("gin-config"),
    ],
)

py_library(
    name = "utils",
    srcs = ["utils.py"],
)

py_binary(
    name = "ppo_balancer",
    srcs = ["main.py"],
    main = "main.py",
    data = glob([
        "policies/*.zip",
    ]),
    deps = [
        "//envs",
        ":rewards",  # needed by gin config
        ":settings",
        "@mpacklog//:python",
        requirement("gin-config"),
        requirement("loop_rate_limiters"),
        requirement("stable_baselines3"),
    ],
)

py_binary(
    name = "train",
    srcs = ["train.py"],
    data = [
        "//spines:bullet",
    ],
    deps = [
        "//envs",
        "//utils:spdlog",
        ":rewards",
        ":settings",
        ":utils",
        "@mpacklog//:python",
        "@rules_python//python/runfiles",
        requirement("gin-config"),
        requirement("stable_baselines3"),
        requirement("torch"),
    ],
)

add_lint_tests()
