# -*- python -*-
#
# Copyright 2022 Stéphane Caron

workspace(name = "upkie_locomotion")

# Repositories
# ============
#
# Add default repositories from leaf to root of the dependency tree.

load("//tools/workspace:default.bzl", "add_default_repositories")
add_default_repositories()

# @vulp is added by add_default_repositories
load("@vulp//tools/workspace:default.bzl", add_vulp_repositories = "add_default_repositories")
add_vulp_repositories()

# @palimpsest is added by add_vulp_repositories
load("@palimpsest//tools/workspace:default.bzl", add_palimpsest_repositories = "add_default_repositories")
add_palimpsest_repositories()

# @pi3hat is added by add_vulp_repositories
load("@pi3hat//tools/workspace:default.bzl", add_pi3hat_repositories = "add_default_repositories")
add_pi3hat_repositories()

# @rpi_bazel is added by add_pi3hat_repositories
load("@rpi_bazel//tools/workspace:default.bzl", add_rpi_bazel_repositories = "add_default_repositories")
add_rpi_bazel_repositories()

# Python dependencies
# ===================

# Depends on @rules_python which is a @palimpsest repository
load("//tools/workspace:parse_python_deps.bzl", "parse_python_deps")
parse_python_deps()
load("@pip_upkie_locomotion//:requirements.bzl", "install_deps")
install_deps()

# Vulp also has Python dependencies
load("@vulp//tools/workspace:install_python_deps.bzl", install_vulp_python_deps = "install_python_deps")
install_vulp_python_deps()
