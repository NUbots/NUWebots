# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import json
import traceback
from types import SimpleNamespace

from logger import logger
from blackboard import blackboard


class Team(SimpleNamespace):
    @classmethod
    def from_json(cls, json_path):
        try:
            with open(json_path) as json_file:
                team = json.load(json_file)
                for field_name in ["name", "players"]:
                    if field_name not in team:
                        raise RuntimeError(f"Missing field {field_name}")
                if len(team['players']) == 0:
                    logger.warning(f"No players found for team {team['name']}")
                count = 1
                for p_key, p in team['players'].items():
                    if int(p_key) != count:
                        raise RuntimeError(f'Wrong team player number: expecting "{count}", found "{p_key}".')
                    for field_name in ['proto', 'halfTimeStartingPose', 'reentryStartingPose', 'shootoutStartingPose',
                                       'goalKeeperStartingPose']:
                        if field_name not in p:
                            raise RuntimeError(f"Missing field {field_name} in player {p_key}")
                    count += 1
        except Exception:
            logger.error(f"Failed to read file {json_path} with the following error:\n{traceback.format_exc()}")
            raise

        return cls(team['name'], team['players'])

    def __init__(self, name, players):
        super().__init__()
        self.name = name
        self.players = players
        self.blackboard = blackboard

    def setup(self):
        # check validity of team files
        # the players IDs should be "1", "2", "3", "4" for four players, "1", "2", "3" for three players, etc.
        for number, player in self.players.items():
            player['outside_circle'] = True
            player['outside_field'] = True
            player['inside_field'] = False
            player['on_outer_line'] = False
            player['inside_own_side'] = False
            player['outside_goal_area'] = True
            player['outside_penalty_area'] = True
            player['left_turf_time'] = None
            # Stores tuples of with (self.sim_time.get_ms()[int], dic) at a 1Hz frequency
            player['history'] = []
            window_size = int(1000 / int(self.blackboard.supervisor.getBasicTimeStep()))  # one second window size
            player['velocity_buffer'] = [[0] * 6] * window_size
            player['ball_handling_start'] = None
            player['ball_handling_last'] = None
