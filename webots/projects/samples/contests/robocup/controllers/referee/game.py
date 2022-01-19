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


from types import SimpleNamespace


class Game(SimpleNamespace):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.red = SimpleNamespace(**self.red)
        self.blue = SimpleNamespace(**self.blue)

        # if the game.json file is malformed with ids defined as string instead of int, we need to convert them to int:
        if not isinstance(self.red.id, int):
            self.red.id = int(self.red.id)
        if not isinstance(self.blue.id, int):
            self.blue.id = int(self.blue.id)

        # set defaults for unset values
        if not hasattr(self, 'minimum_real_time_factor'):
            self.minimum_real_time_factor = 3  # we guarantee that each time step lasts at least 3x simulated time
        if not hasattr(self, 'press_a_key_to_terminate'):
            self.press_a_key_to_terminate = False

        self.penalty_shootout = self.type == 'PENALTY'
        self.penalty_shootout_count = 0
        self.penalty_shootout_goal = False
        self.penalty_shootout_time_to_score = [None, None, None, None, None, None, None, None, None, None]
        self.penalty_shootout_time_to_reach_goal_area = [None, None, None, None, None, None, None, None, None, None]
        self.penalty_shootout_time_to_touch_ball = [None, None, None, None, None, None, None, None, None, None]
        self.ball_radius = 0.07 if self.field_size == 'kid' else 0.1125
        # initial position of ball before kick
        self.ball_kick_translation = [0, 0,
                                      self.ball_radius + self.blackboard.field.turf_depth]
        self.ball_exit_translation = None
        self.ball_last_touch_time = 0
        self.ball_first_touch_time = 0
        self.ball_last_touch_time_for_display = 0
        self.ball_position = [0, 0, 0]
        self.ball_last_move = 0
        self.real_time_multiplier = 1000 / (
                self.minimum_real_time_factor * int(self.blackboard.supervisor.getBasicTimeStep())) \
            if self.minimum_real_time_factor > 0 else 10
        self.interruption = None
        self.interruption_countdown = 0
        self.interruption_step = None
        self.interruption_step_time = 0
        self.interruption_team = None
        self.interruption_seconds = None
        self.dropped_ball = False
        self.overtime = False
        self.finished_overtime = False
        self.ready_countdown = 0  # simulated time countdown before ready state (used in kick-off after goal and dropped ball)
        self.play_countdown = 0
        self.in_play = None
        self.throw_in = False  # True while throwing in to allow ball handling
        self.throw_in_ball_was_lifted = False  # True if the throwing-in player lifted the ball
        self.over = False
        self.wait_for_state = 'INITIAL'
        self.wait_for_sec_state = None
        self.wait_for_sec_phase = None

    def reset_ball_touched(self):
        self.ball_previous_touch_team = None
        self.ball_previous_touch_player_number = None
        self.ball_last_touch_team = None
        self.ball_last_touch_player_number = None

    def set_ball_touched(self, team_color, player_number):
        self.ball_previous_touch_team = self.ball_last_touch_team
        self.ball_previous_touch_player_number = self.ball_last_touch_player_number
        self.ball_last_touch_team = team_color
        self.ball_last_touch_player_number = player_number
        self.dropped_ball = False

    def set_penalty(self, team_color, ball_x):
        self.in_play = None
        self.can_score = True
        self.can_score_own = False
        self.ball_set_kick = True
        self.ball_left_circle = True
        self.ball_must_kick_team = team_color
        self.kicking_player_number = None
        self.ball_kick_translation[0] = ball_x
        self.ball_kick_translation[1] = 0
        self.ball_translation.setSFVec3f(self.ball_kick_translation)

    def set_kickoff(self, team_color):
        self.phase = 'KICKOFF'
        self.ball_kick_translation[0] = 0
        self.ball_kick_translation[1] = 0
        self.ball_set_kick = True
        self.ball_first_touch_time = 0
        self.in_play = None
        self.ball_must_kick_team = team_color
        self.reset_ball_touched()
        self.ball_left_circle = None  # one can score only after ball went out of the circle
        self.can_score = False  # or was touched by another player
        self.can_score_own = False
        self.kicking_player_number = None

    def set_dropped_ball(self):
        self.phase = 'DROPPEDBALL'
        self.ball_kick_translation[0] = 0
        self.ball_kick_translation[1] = 0
        self.ball_set_kick = True
        self.ball_first_touch_time = 0
        self.in_play = None
        self.dropped_ball = True
        self.can_score = True
        self.can_score_own = False

    def ball_in_play(self, ms):
        self.in_play = ms
        self.ball_last_move = ms
        self.interruption = None
        self.interruption_step = None
        self.interruption_step_time = 0
        self.interruption_team = None
        self.interruption_seconds = None
