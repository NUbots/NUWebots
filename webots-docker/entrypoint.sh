#!/bin/bash
python3 docker-config.py ${HOME}/hlvs_webots/controllers/referee/game.json
rm docker-config.py
# Run the command given by CMD or docker run parameter, replacing current process
exec "$@"
