# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

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
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
    ],
)

py_binary(
    name = "bullet",
    srcs = ["bullet.py"],
    main = "bullet.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        ":config",
        "//spines:bullet_spine",
    ],
    deps = [
        "//upkie/utils:exceptions",
        ":controllers",
        "@rules_python//python/runfiles",
        "//upkie:python",
    ] + select({
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("gin-config"),
            requirement("loop-rate-limiters"),
            requirement("numpy"),
        ],
    }),
)

py_binary(
    name = "pid_balancer",
    srcs = ["main.py"],
    main = "main.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        "//pid_balancer/config",
    ] + select({
        "//:pi64_config": ["//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/utils:exceptions",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
        ":controllers",
        "//upkie:python",
    ],
)

add_lint_tests()
