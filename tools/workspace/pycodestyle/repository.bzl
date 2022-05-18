# -*- python -*-

load("//tools/workspace:github_archive.bzl", "github_archive")

def pycodestyle_repository():
    """
    Download repository from GitHub as a ZIP archive, decompress it, and make
    its targets available for binding.
    """
    github_archive(
        name = "pycodestyle",
        repository = "PyCQA/pycodestyle",
        commit = "2.7.0",
        sha256 = "9a28acc352f29c770630745ddb77968d53bc544d8b173a11f16756db8aa0a1ba",
        build_file = Label("//tools/workspace/pycodestyle:package.BUILD"),
    )
