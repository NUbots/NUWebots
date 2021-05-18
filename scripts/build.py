#!/usr/bin/env python3

import os
import subprocess
from multiprocessing import cpu_count

from termcolor import cprint

import b


def register(command):
    # Install help
    command.help = "build the codebase"

    command.add_argument("args", nargs="...", help="the arguments to pass through to ninja")
    command.add_argument("-j", help="number of jobs to spawn")


def run(j, args, **kwargs):
    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "build"))

    # Run cmake if we don't have a ninja build file
    if not os.path.isfile("build.ninja"):
        exitcode = os.system("cmake {} -GNinja".format(b.project_dir)) >> 8

        # If cmake errors return with its status
        if exitcode != 0:
            exit(exitcode)

    # Default to using all cores
    if j is None:
        j = cpu_count()

    command = ["ninja", "-j{}".format(j), *args]

    # Return the exit code of make
    exit(subprocess.run(command).returncode)
