#!/usr/bin/env python3

import multiprocessing
import os
import sys
from functools import partial
from subprocess import STDOUT, CalledProcessError, check_output

from termcolor import cprint

import b


def _do_format(path):
    text = ""
    try:
        # Use the absolute path to file
        abs_path = os.path.abspath(path)

        if path.endswith((".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".proto")) and "protos" not in path:
            text = "Formatting {} with clang-format\n".format(path)
            text = text + check_output(["clang-format", "-i", "-style=file", abs_path], stderr=STDOUT).decode("utf-8")
        elif path.endswith(("CMakeLists.txt", ".cmake")):
            text = "Formatting {} with cmake-format\n".format(path)
            text = text + check_output(["cmake-format", "-i", abs_path], stderr=STDOUT).decode("utf-8")
        elif path.endswith((".py")):
            text = "Formatting {} with isort and black\n".format(path)
            text = text + check_output(["isort", abs_path], stderr=STDOUT).decode("utf-8")
            text = text + check_output(["black", abs_path], stderr=STDOUT).decode("utf-8")
    except CalledProcessError as e:
        text = text + e.output.decode("utf-8")
    except:
        cprint(
            "You don't have one of the formatters installed.\nRun `sudo -H pip3 install -r requirements.txt`",
            color="red",
        )

    return text


def register(command):
    command.help = "Format all the code in the codebase using clang-format"


def run(**kwargs):
    # Change into the project directory
    os.chdir(b.project_dir)

    # Use git to get all of the files that are committed to the repository
    # Do not format files in the webots directory
    files = check_output(["git", "ls-files", "--", ".", ":!:webots/*"]).decode("utf-8")

    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        for r in pool.imap_unordered(partial(_do_format), files.splitlines()):
            sys.stdout.write(r)
