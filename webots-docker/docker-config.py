import json, os, sys

TEAM_NAME = os.environ.get('TEAM_NAME')
ROBOT_HOST = os.environ.get('ROBOT_HOST')

print(type(ROBOT_HOST))

filename = sys.argv[1]

with open(filename, 'r') as f:
    game_config = json.load(f)

if TEAM_NAME: game_config['red']['config'] = TEAM_NAME + '.json'
if ROBOT_HOST: game_config['red']['hosts'].append(ROBOT_HOST)

with open(filename, 'w') as f:
    json.dump(game_config, f, indent=2)
