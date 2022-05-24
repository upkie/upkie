# -*- python -*-
#
# Copyright 2022 Stéphane Caron

workspace(name = "upkie_locomotion")

BAZEL_VERSION = "4.1.0"
BAZEL_VERSION_SHA_HOST = "0eb2e378d2782e7810753e2162245ad1179c1bb12f848c692b4a595b4edf779b"
BAZEL_VERSION_SHA_PI = "02fcc51686a2f7b360a629747134d62dec885012454fac4c8634fc525884201f"

load("//tools/workspace:default.bzl", "add_default_repositories")
add_default_repositories()

# @vulp is added from tools/workspace
load("@vulp//tools/workspace:default.bzl", add_vulp_repositories = "add_default_repositories")
add_vulp_repositories()

# @pi3hat is added from @vulp
load("@pi3hat//tools/workspace:default.bzl", add_pi3hat_repositories = "add_default_repositories")
add_pi3hat_repositories()

# @rpi_bazel is added from @pi3hat
load("@rpi_bazel//tools/workspace:default.bzl", add_rpi_bazel_repositories = "add_default_repositories")
add_rpi_bazel_repositories()

# Python packages
load("@rules_python//python:pip.bzl", "pip_install")
pip_install(
    name = "pip",
    requirements = "//:requirements.txt",
)
