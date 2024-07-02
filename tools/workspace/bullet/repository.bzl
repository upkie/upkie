# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/workspace:github_archive.bzl", "github_archive")

def bullet_repository(**kwargs):
    """
    Make the Bullet target available for binding.
    """
    github_archive(
        name = "bullet",
        repository = "bulletphysics/bullet3",
        commit = "3.24",
        sha256 = "1179bcc5cdaf7f73f92f5e8495eaadd6a7216e78cad22f1027e9ce49b7a0bfbe",
        build_file = Label("//tools/workspace/bullet:package.BUILD"),
    )
