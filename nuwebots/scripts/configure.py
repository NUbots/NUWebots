#!/usr/bin/env python3

import os
import subprocess

import b


def register(command):
    # Install help
    command.help = "Configure the project in a docker container"

    command.add_argument(
        "--clean",
        dest="purge_build_folder",
        action="store_true",
        default=False,
        help="Purge the build folder before configuring",
    )

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )
    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


def run(purge_build_folder, interactive, args, **kwargs):

    # Purge the build folder if we have passed the purge_build_folder argument
    if purge_build_folder:
        from shutil import rmtree

        # We ignore errors so that it doesn't fail if `build` doesn't exist
        rmtree(b.build_dir, ignore_errors=True)

    # Make sure we have a build directory, then change into it
    os.makedirs(b.build_dir, exist_ok=True)
    os.chdir(b.build_dir)

    # If interactive then run ccmake else just run cmake
    command = ["ccmake" if interactive else "cmake"]

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    command.extend([*args, b.project_dir, "-GNinja"])

    exit(subprocess.run(command).returncode)
