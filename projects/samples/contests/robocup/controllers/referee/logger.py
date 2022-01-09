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


import time
from controller import AnsiCodes
import sys

from blackboard import blackboard


class Logger:
    def __init__(self):
        self.blackboard = blackboard
        self.log_file = open('log.txt', 'w')

    def log(self, message, msg_type, force_flush=True):
        if type(message) is list:
            for m in message:
                self.log(m, msg_type, False)
            if self.log_file and force_flush:
                self.log_file.flush()
            return
        if msg_type == 'Warning':
            console_message = f'{AnsiCodes.YELLOW_FOREGROUND}{AnsiCodes.BOLD}{message}{AnsiCodes.RESET}'
        elif msg_type == 'Error':
            console_message = f'{AnsiCodes.RED_FOREGROUND}{AnsiCodes.BOLD}{message}{AnsiCodes.RESET}'
        else:
            console_message = message
        print(console_message, file=sys.stderr if msg_type == 'Error' else sys.stdout)
        if self.log_file:
            real_time = int(1000 * (time.time() - self.blackboard.start_real_time)) / 1000
            # log real and virtual times
            self.log_file.write(f'[{real_time:08.3f}|{self.blackboard.sim_time.get_ms() / 1000:08.3f}] {msg_type}: {message}\n')
            if force_flush:
                self.log_file.flush()

    def info(self, message):
        self.log(message, 'Info')

    def warning(self, message):
        self.log(message, 'Warning')

    def error(self, message, fatal=False):
        self.log(message, 'Error')

    def close(self):
        self.log_file.close()

logger = Logger()
