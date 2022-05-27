# -*- python -*-
#
# Copyright 2022 Stéphane Caron

workspace(name = "upkie_locomotion")

BAZEL_VERSION = "4.1.0"
BAZEL_VERSION_SHA_HOST = "0eb2e378d2782e7810753e2162245ad1179c1bb12f848c692b4a595b4edf779b"
BAZEL_VERSION_SHA_PI = "02fcc51686a2f7b360a629747134d62dec885012454fac4c8634fc525884201f"

load("//tools/workspace:default.bzl", "add_default_repositories")
add_default_repositories()

# @vulp is added by add_default_repositories
load("@vulp//tools/workspace:default.bzl", add_vulp_repositories = "add_default_repositories")
add_vulp_repositories()

# Depends on @rules_python which is added by add_default_repositories
load("//tools/workspace:install_python_deps.bzl", "install_python_deps")
install_python_deps()

# Vulp also has Python dependencies
load("@vulp//tools/workspace:install_python_deps.bzl", install_vulp_python_deps = "install_python_deps")
install_vulp_python_deps()

# @pi3hat is added by add_vulp_repositories
load("@pi3hat//tools/workspace:default.bzl", add_pi3hat_repositories = "add_default_repositories")
add_pi3hat_repositories()

# @rpi_bazel is added by add_pi3hat_repositories
load("@rpi_bazel//tools/workspace:default.bzl", add_rpi_bazel_repositories = "add_default_repositories")
add_rpi_bazel_repositories()
