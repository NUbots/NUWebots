import json, os, sys

TEAM_NAME = os.environ.get('TEAM_NAME')
ROBOT_HOSTS = os.environ.get('ROBOT_HOSTS')

filename = sys.argv[1]

with open(filename, 'r') as f:
    game_config = json.load(f)

if TEAM_NAME: game_config['red']['config'] = 'teams/' + TEAM_NAME + '.json'
if ROBOT_HOSTS: game_config['red']['hosts'] += ROBOT_HOSTS.split(",")

with open(filename, 'w') as f:
    json.dump(game_config, f, indent=2)
