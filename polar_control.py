#
# simple polar control
#

import math

from position_control import *


class PolarControl(PositionControl):

    def __init__(self, motion, kp_lin, v_max, kp_angular, w_max, threshold=0.005):
        super(PolarControl, self).__init__(motion)
        self.kp_lin = kp_lin
        self.v_max = v_max
        self.kp_angular = kp_angular
        self.w_max = w_max
        self.motion = motion
        self.threshold = threshold  # 5mm

    def set_target(self, x, y):
        self.reset()
        self._target_got = False
        self.target_x = x
        self.target_y = y

    def evaluate(self, delta_t):

        dx = self.target_x - self.motion.x
        dy = self.target_y - self.motion.y

        target_heading = math.atan2(dy, dx)

        distance = math.hypot(dx, dy)
        if distance <= self.threshold:
            self._target_got = True
            self.motion.evaluate(0.0, 0.0, delta_t)
        else:

            self._target_got = False
            heading_error = target_heading - self.motion.theta

            # Mia aggiunta.
            # Quando motion.theta si trova in prossimità del pi cambia di segno repentinamente
            # Questo porta ad avere valori di target heading e motion.theta di valore quasi identico
            # ad avere segno discorde, e dunque a sommarsi al posto di sottrarsi. Un errore prossimo a
            # 0 diventa un errore di circa 2pi. Ho aggiunto quindi il filtro di cui dispone già motion.theta
            # anche all'errore calcolato

            if heading_error > math.pi:
                heading_error = heading_error - 2 * math.pi
            if heading_error < -math.pi:
                heading_error = 2 * math.pi + heading_error

            v = self.kp_lin * distance
            w = self.kp_angular * heading_error

            if v > self.v_max:
                v = self.v_max
            elif v < -self.v_max:
                v = - self.v_max

            if w > self.w_max:
                w = self.w_max
            elif w < -self.w_max:
                w = - self.w_max

            # v = w = 0.1
            #print('polar control velocities', v, w)
            self.motion.evaluate_vw(v, w, delta_t, True)

    def evaluate_d(self, distance, delta_t):
        v = self.kp_lin * distance
        if v > self.v_max:
            v = self.v_max
        elif v < -self.v_max:
            v = - self.v_max
        self.motion.evaluate_vw(v, 0.0, delta_t, True)
