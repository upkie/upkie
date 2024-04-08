load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "controllers",
    srcs = [
        "servo_controller.py",
        "wheel_controller.py",
    ],
    deps = [
        "//upkie/observers/base_pitch",
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
    ],
)


py_binary(
    name = "lessie_balancer",
    srcs = ["main.py"],
    main = "main.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        "//lessie_balancer/config",
    ] + select({
        "//:pi64_config": ["//spines:lessie_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/utils:exceptions",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
        ":controllers",
        "@vulp//:python",
    ],
)

add_lint_tests()
