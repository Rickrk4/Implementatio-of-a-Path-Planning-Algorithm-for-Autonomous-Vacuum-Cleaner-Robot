#
# position_control.py
#

import math

class PositionControl:

    def __init__(self, motion):
        self.motion = motion
        self._target_got = False

    def reset(self):
        self._target_got = False

    def target_got(self):
        return self._target_got

