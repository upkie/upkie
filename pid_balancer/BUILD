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
    srcs = [
        "bullet.py"
    ],
    main = "bullet.py",
    data = [
        ":config",
        "//spines:bullet_spine",
    ],
    deps = [
        "//upkie",
        ":controllers",
        "@rules_python//python/runfiles",
        requirement("PyYAML"),
        requirement("gin-config"),
        requirement("loop-rate-limiters"),
    ] + select({
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
        ],
    }),
)

py_binary(
    name = "pid_balancer",
    srcs = [
        "main.py",
    ],
    main = "main.py",
    data = [
        "//pid_balancer/config",
    ] + select({
        "//:pi64_config": ["//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie",
        ":controllers",
        requirement("PyYAML"),
        requirement("gin-config"),
        requirement("loop-rate-limiters"),
    ] + select({
        "//:pi64_config": [],
        "//conditions:default": [
            requirement("numpy"),
        ],
    }),
)

add_lint_tests()
