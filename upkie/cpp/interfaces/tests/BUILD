# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "imu_uncertainty_test",
    srcs = [
        "ImuUncertaintyTest.cpp",
    ],
    deps = [
        "//upkie/cpp/actuation:imu",
        "@googletest//:main",
    ],
)

cc_test(
    name = "interface_test",
    srcs = [
        "InterfaceTest.cpp",
    ],
    deps = [
        "//upkie/cpp/actuation:interface",
        "@googletest//:main",
    ],
)

cc_test(
    name = "mock_interface_test",
    srcs = [
        "MockInterfaceTest.cpp",
    ],
    deps = [
        "//upkie/cpp/actuation:mock_interface",
        "@googletest//:main",
    ],
)

cc_test(
    name = "bullet_interface_test",
    srcs = [
        "BulletInterfaceTest.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/cpp/actuation:bullet_interface",
        "@bazel_tools//tools/cpp/runfiles",
        "@googletest//:main",
    ],
)

cc_test(
    name = "pi3hat_interface_test",
    srcs = select({
        "//:pi64_config": ["Pi3HatInterfaceTest.cpp"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/cpp/actuation:pi3hat_interface",
        "@bazel_tools//tools/cpp/runfiles",
        "@googletest//:main",
    ],
)

cc_test(
    name = "bullet_interface_env_bodies",
    srcs = [
        "BulletInterfaceEnvBodies.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//upkie/cpp/actuation:bullet_interface",
        "@bazel_tools//tools/cpp/runfiles",
        "@googletest//:main",
    ],
)

add_lint_tests()
