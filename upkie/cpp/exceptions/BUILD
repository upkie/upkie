# -*- python -*-

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "filter_error",
    hdrs = ["FilterError.h"],
)

cc_library(
    name = "observer_error",
    hdrs = ["ObserverError.h"],
)

cc_library(
    name = "exceptions",
    hdrs = [
        "TypeError.h",
        "UpkieError.h",
    ],
    deps = [
        ":filter_error",
        ":observer_error",
    ],
)

add_lint_tests()
