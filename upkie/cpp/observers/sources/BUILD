# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cpu_temperature",
    hdrs = ["CpuTemperature.h"],
    srcs = ["CpuTemperature.cpp"],
    deps = [
        "//vulp/observation:source",
    ],
)

cc_library(
    name = "joystick",
    hdrs = select({
        "@//:linux": ["Joystick.h"],
        "@//conditions:default": [],
    }),
    srcs = select({
        "@//:linux": ["Joystick.cpp"],
        "@//conditions:default": [],
    }),
    deps = [
        "//vulp/observation:source",
    ],
)

cc_library(
    name = "keyboard",
    hdrs = ["Keyboard.h"],
    srcs = ["Keyboard.cpp"],
    deps = [
        "//vulp/observation:source",
    ],
)

cc_library(
    name = "sources",
    deps =  select({
        "@//:linux": [":joystick", ":cpu_temperature", ":keyboard"],
        "@//conditions:default": [":cpu_temperature", ":keyboard"],
    }),
)

add_lint_tests()
