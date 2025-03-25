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
        ":sensor",
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
        ":sensor",
    ],
)

cc_library(
    name = "keyboard",
    hdrs = ["Keyboard.h"],
    srcs = ["Keyboard.cpp"],
    deps = [
        ":sensor",
    ],
)

cc_library(
    name = "sensor",
    hdrs = ["Sensor.h"],
    deps = [
        "@palimpsest",
    ],
)

cc_library(
    name = "sensor_pipeline",
    hdrs = ["SensorPipeline.h"],
    srcs = [
        "SensorPipeline.cpp",
    ],
    deps = [
        ":sensor",
    ],
)

cc_library(
    name = "sensors",
    deps =  [
        ":sensor_pipeline",
    ] + select({
        "@//:linux": [":joystick", ":cpu_temperature", ":keyboard"],
        "@//conditions:default": [":cpu_temperature", ":keyboard"],
    }),
)

add_lint_tests()
