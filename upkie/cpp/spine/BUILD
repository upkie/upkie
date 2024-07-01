# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "agent_interface",
    hdrs = [
        "AgentInterface.h",
        "Request.h",
    ],
    srcs = [
        "AgentInterface.cpp",
    ],
    deps = [
        "@spdlog",
    ],
)

cc_library(
    name = "state_machine",
    hdrs = [
        "StateMachine.h",
    ],
    srcs = [
        "StateMachine.cpp",
    ],
    deps = [
        ":agent_interface",
        "//upkie/cpp/utils:handle_interrupts",
    ],
)

cc_library(
    name = "spine",
    hdrs = [
        "Spine.h",
    ],
    srcs = [
        "Spine.cpp",
    ],
    linkopts = select({
        "@//:linux": ["-lrt"],
        "@//conditions:default": [],
    }),
    deps = [
        "//upkie/cpp/actuation:interface",
        "//upkie/cpp/exceptions",
        "//upkie/cpp/observers:observe_servos",
        "//upkie/cpp/observers:observe_time",
        "//upkie/cpp/observers:observer_pipeline",
        "//upkie/cpp/utils:realtime",
        "//upkie/cpp/utils:synchronous_clock",
        ":state_machine",
        "@mpacklog",
    ],
)

py_library(
    name = "python",
    srcs = [
        "__init__.py",
        "exceptions.py",
        "request.py",
        "spine_interface.py",
    ],
)

add_lint_tests()
