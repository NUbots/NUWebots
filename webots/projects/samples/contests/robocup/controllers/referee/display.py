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
from blackboard import blackboard


class Display:
    def __init__(self):
        self.blackboard = blackboard
        self.font_size = blackboard.config.FONT_SIZE
        self.font = blackboard.config.FONT

    def update_time_display(self):
        if self.blackboard.game.state:
            s = self.blackboard.game.state.seconds_remaining
            value = format_time(s)
        else:
            value = ' --:--'
        self.blackboard.supervisor.setLabel(6, value, 0, 0, self.font_size, 0x000000, 0.2, self.font)

    def update_state_display(self):
        if self.blackboard.game.state:
            state = self.blackboard.game.state.game_state[6:]
            if state == 'READY' or state == 'SET':  # kickoff
                if self.blackboard.game.kickoff == self.blackboard.game.red.id:
                    color = self.blackboard.config.RED_COLOR
                else:
                    color = self.blackboard.config.BLUE_COLOR
            else:
                color = 0x000000
        else:
            state = ''
            color = 0x000000
        self.blackboard.supervisor.setLabel(7, ' ' * 41 + state, 0, 0, self.font_size, color, 0.2, self.font)
        self.update_details_display()

    def update_score_display(self):
        if self.blackboard.game.state:
            red = 0 if self.blackboard.game.state.teams[0].team_color == 'RED' else 1
            blue = 1 if red == 0 else 0
            red_score = str(self.blackboard.game.state.teams[red].score)
            blue_score = str(self.blackboard.game.state.teams[blue].score)
        else:
            red_score = '0'
            blue_score = '0'
        if self.blackboard.game.side_left == self.blackboard.game.blue.id:
            offset = 21 if len(blue_score) == 2 else 22
            score = ' ' * offset + blue_score + '-' + red_score
        else:
            offset = 21 if len(red_score) == 2 else 22
            score = ' ' * offset + red_score + '-' + blue_score
        self.blackboard.supervisor.setLabel(5, score, 0, 0, self.font_size, self.blackboard.config.BLACK_COLOR, 0.2, self.font)

    def update_team_details_display(self, team, side, strings):
        for n in range(len(team.players)):
            robot_info = self.blackboard.game.state.teams[side].players[n]
            strings.background += '█  '
            if robot_info.number_of_warnings > 0:  # a robot can have both a warning and a yellow card
                strings.warning += '■  '
                strings.yellow_card += ' ■ ' if robot_info.number_of_yellow_cards > 0 else '   '
            else:
                strings.warning += '   '
                strings.yellow_card += '■  ' if robot_info.number_of_yellow_cards > 0 else '   '
            strings.red_card += '■  ' if robot_info.number_of_red_cards > 0 else '   '
            strings.white += str(n + 1) + '██'
            strings.foreground += f'{robot_info.secs_till_unpenalized:02d} ' \
                if robot_info.secs_till_unpenalized != 0 else '   '

    def update_details_display(self):
        if not self.blackboard.game.state:
            return
        red = 0 if self.blackboard.game.state.teams[0].team_color == 'RED' else 1
        blue = 1 if red == 0 else 0
        if self.blackboard.game.side_left == self.blackboard.game.red.id:
            left = red
            right = blue
            left_team = self.blackboard.red_team
            right_team = self.blackboard.blue_team
            left_color = self.blackboard.config.RED_COLOR
            right_color = self.blackboard.config.BLUE_COLOR
        else:
            left = blue
            right = red
            left_team = self.blackboard.blue_team
            right_team = self.blackboard.red_team
            left_color = self.blackboard.config.BLUE_COLOR
            right_color = self.blackboard.config.RED_COLOR

        strings = SimpleNamespace()
        strings.foreground = ' ' + format_time(self.blackboard.game.state.secondary_seconds_remaining) + '  ' \
            if self.blackboard.game.state.secondary_seconds_remaining > 0 else ' ' * 8
        strings.background = ' ' * 7
        strings.warning = strings.background
        strings.yellow_card = strings.background
        strings.red_card = strings.background
        strings.white = '█' * 7
        self.update_team_details_display(left_team, left, strings)
        strings.left_background = strings.background
        strings.background = ' ' * 28
        space = 21 - len(left_team.players) * 3
        strings.white += '█' * space
        strings.warning += ' ' * space
        strings.yellow_card += ' ' * space
        strings.red_card += ' ' * space
        strings.foreground += ' ' * space
        self.update_team_details_display(right_team, right, strings)
        strings.right_background = strings.background
        del strings.background
        space = 12 - 3 * len(right_team.players)
        strings.white += '█' * (22 + space)
        strings.secondary_state = ' ' * 41 + self.blackboard.game.state.secondary_state[6:]
        sr = self.blackboard.config.IN_PLAY_TIMEOUT - self.blackboard.game.interruption_seconds \
            + self.blackboard.game.state.seconds_remaining \
            if self.blackboard.game.interruption_seconds is not None else 0
        if sr > 0:
            strings.secondary_state += ' ' + format_time(sr)
        if (self.blackboard.game.state.secondary_state[6:] != 'NORMAL' or
                self.blackboard.game.state.secondary_state_info[1] != 0):
            strings.secondary_state += ' [' + str(self.blackboard.game.state.secondary_state_info[1]) + ']'
        if self.blackboard.game.interruption_team is not None:  # interruption
            secondary_state_color = self.blackboard.config.RED_COLOR \
                if self.blackboard.game.interruption_team == self.blackboard.game.red.id else self.blackboard.config.BLUE_COLOR
        else:
            secondary_state_color = self.blackboard.config.BLACK_COLOR
        y = 0.0465  # vertical position of the second line
        self.blackboard.supervisor.setLabel(10, strings.left_background, 0, y, self.font_size, left_color, 0.2, self.font)
        self.blackboard.supervisor.setLabel(11, strings.right_background, 0, y, self.font_size, right_color, 0.2, self.font)
        self.blackboard.supervisor.setLabel(12, strings.white, 0, y, self.font_size,
                                            self.blackboard.config.WHITE_COLOR, 0.2, self.font)
        self.blackboard.supervisor.setLabel(13, strings.warning, 0, 2 * y, self.font_size, 0x0000ff, 0.2, self.font)
        self.blackboard.supervisor.setLabel(14, strings.yellow_card, 0, 2 * y, self.font_size, 0xffff00, 0.2, self.font)
        self.blackboard.supervisor.setLabel(15, strings.red_card, 0, 2 * y, self.font_size, 0xff0000, 0.2, self.font)
        self.blackboard.supervisor.setLabel(16, strings.foreground, 0, y, self.font_size,
                                            self.blackboard.config.BLACK_COLOR, 0.2, self.font)
        self.blackboard.supervisor.setLabel(17, strings.secondary_state, 0, y, self.font_size,
                                            secondary_state_color, 0.2, self.font)

    def update_team_display(self):
        # red and blue backgrounds
        left_color = self.blackboard.config.RED_COLOR \
            if self.blackboard.game.side_left == self.blackboard.game.red.id else self.blackboard.config.BLUE_COLOR
        right_color = self.blackboard.config.BLUE_COLOR \
            if self.blackboard.game.side_left == self.blackboard.game.red.id else self.blackboard.config.RED_COLOR
        self.blackboard.supervisor.setLabel(2, ' ' * 7 + '█' * 14, 0, 0, self.font_size, left_color, 0.2, self.font)
        self.blackboard.supervisor.setLabel(3, ' ' * 26 + '█' * 14, 0, 0, self.font_size, right_color, 0.2, self.font)
        # white background and names
        left_team = self.blackboard.red_team \
            if self.blackboard.game.side_left == self.blackboard.game.red.id else self.blackboard.blue_team
        right_team = self.blackboard.red_team \
            if self.blackboard.game.side_left == self.blackboard.game.blue.id else self.blackboard.blue_team
        team_names = 7 * '█' + (13 - len(left_team.name)) * ' ' + left_team.name + \
            ' █████ ' + right_team.name + ' ' * (13 - len(right_team.name)) + '█' * 22
        self.blackboard.supervisor.setLabel(4, team_names, 0, 0, self.font_size,
                                            self.blackboard.config.WHITE_COLOR, 0.2, self.font)
        self.update_score_display()

    def setup_display(self):
        self.update_team_display()
        self.update_time_display()
        self.update_state_display()


def format_time(s):
    if s < 0:
        s = -s
        sign = '-'
    else:
        sign = ' '
    seconds = str(s % 60)
    minutes = str(int(s / 60))
    if len(minutes) == 1:
        minutes = '0' + minutes
    if len(seconds) == 1:
        seconds = '0' + seconds
    return sign + minutes + ':' + seconds
