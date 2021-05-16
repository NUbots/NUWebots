#!/usr/bin/env python3

import os
import subprocess

import b


def register(command):
    # Install help
    command.help = "Configure the project in a docker container"

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )
    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


def run(interactive, args, **kwargs):

    # Make sure we have a build directory
    try:
        os.makedirs("build")
    except FileExistsError:
        # directory already exists
        pass

    # If interactive then run ccmake else just run cmake
    os.chdir(os.path.join(b.project_dir, "build"))

    command = ["ccmake" if interactive else "cmake"]

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    command.extend([*args, b.project_dir])

    exit(subprocess.run(command).returncode)
