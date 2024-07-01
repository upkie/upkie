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
        "//vulp/utils:handle_interrupts",
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
        "//vulp/actuation:interface",
        "//vulp/exceptions",
        "//vulp/observation:observe_servos",
        "//vulp/observation:observe_time",
        "//vulp/observation:observer_pipeline",
        "//vulp/utils:realtime",
        "//vulp/utils:synchronous_clock",
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
