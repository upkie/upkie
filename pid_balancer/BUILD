# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "modules",
    srcs = [
        "imu_placement.py",
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
        "main_bullet.py"
    ],
    main = "main_bullet.py",
    data = [
        ":config",
        "//spines:bullet_spine",
    ],
    deps = [
        "//upkie",
        ":modules",
        "@rules_python//python/runfiles",
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
        "main_standalone.py",
    ],
    main = "main_standalone.py",
    data = [
        "//pid_balancer/config",
    ] + select({
        "//:pi64_config": ["//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie",
        ":modules",
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
