#
# simple polar control
#

import math

from position_control import *


class SmoothVWControl(PositionControl):

    def __init__(self, motion, kp_lin, v_max, sensor, threshold=0.05):
        super(SmoothVWControl, self).__init__(motion)
        self.kp_lin = kp_lin
        self.v_max = v_max
        self.sensor = sensor
        self.motion = motion
        self.threshold = threshold  # 5mm
        self._target_got = False

    def get_distance(self):
        return self.sensor.evaluate()

    def set_target(self, v, w):
        self.target_v = v
        self.target_w = w

    def evaluate(self, delta_t):

        v, w = self.motion.get_speeds_vw()


        if abs(v - self.target_v) < self.threshold and abs(w - self.target_w) < self.threshold: #and self.motion.get_speeds()[0] < self.threshold and self.motion.get_speeds()[1] < self.threshold:
            self._target_got = True
            self.motion.evaluate(self.target_v, self.target_w, delta_t)
        else:
            self._target_got = False

            v = v + self.target_v / 2
            w = w + self.target_w / 2

            if v > self.v_max:
                v = self.v_max
            elif v < -self.v_max:
                v = - self.v_max

            self.motion.evaluate_vw(v, w, delta_t)
