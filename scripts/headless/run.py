#!/user/bin/env python3

import subprocess

from termcolor import cprint

import b

def register(command):
    command.help = "run the headless version of webots"

def run(**kwargs):
    err = subprocess.run(
        [
            "docker",
            "run",
            "--name",
            "webots_headless_runner",
            "--attach",
            "stdin",
            "--attach",
            "stdout",
            "--attach",
            "stderr",
            "-e",
            "AUDIODEV=null",
            "-e",
            "DEBIAN_FRONTEND=noninteractive",
            "-e",
            "DISPLAY=:99",
            "-e",
            "LIBGL_ALWAYS_SOFTWARE=true",
            "webots_headless",
            "Xvfb :99 -screen 0 1024x768x16 & webots",
        ]
    )
