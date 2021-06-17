#!/user/bin/env python3

import subprocess

from termcolor import cprint

import b

def register(command):
    command.help = "build the headless version of webots"

def run(**kwargs):
    err = subprocess.run(
        [
            "docker",
            "build",
            "-t",
            "webots_headless",
            "docker",
        ]
    )
