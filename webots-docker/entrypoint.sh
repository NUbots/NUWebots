#!/bin/bash

python3 webots-config.py /home/webots/hlvs_webots/controllers/referee/game.json
# Run the command given by CMD or docker run parameter, replacing current process
exec "$@"
