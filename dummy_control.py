#
# simple polar control
#

import math

from position_control import *


class DummyControl(PositionControl):

    def __init__(self, motion, kp_lin, v_max, sensor, threshold=0.05):
        super(DummyControl, self).__init__(motion)
        self.kp_lin = kp_lin
        self.v_max = v_max
        self.sensor = sensor
        self.motion = motion
        self.threshold = threshold  # 5mm
        self._target_got = False

    def get_distance(self):
        return self.sensor.evaluate()

    def evaluate(self, delta_t):

        distance = self.get_distance()

        #print('d=', distance, 'th=',self.threshold)


        if distance <= self.threshold: #and self.motion.get_speeds()[0] < self.threshold and self.motion.get_speeds()[1] < self.threshold:
            self._target_got = True
            self.motion.evaluate(0.0, 0.0, delta_t)
        else:
            self._target_got = False

            v = self.kp_lin * distance
            w = 0

            if v > self.v_max:
                v = self.v_max
            elif v < -self.v_max:
                v = - self.v_max

            self.motion.evaluate_vw(v, w, delta_t)
