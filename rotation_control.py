#
# rotation_control.py
#

import math
from geometry import *
from position_control import *


class AbsoluteRotationProportionalControl(PositionControl):

    def __init__(self, motion, _kp, _w_max):
        super(AbsoluteRotationProportionalControl, self).__init__(motion)
        self.kp = _kp
        self.w_max = _w_max
        self.threshold = math.radians(2)

    def set_target(self, target_heading):
        self.reset()
        self.target_heading = target_heading

    def evaluate(self, delta_t):
        heading_error = normalize_angle_radians(math.radians(self.target_heading) - self.motion.theta)

        if (abs(heading_error) < self.threshold) or (self.target_got):
            self.target_got = True
            self.motion.evaluate_vw(0, 0, delta_t)
        else:
            self.target_got = False
            w = self.kp * heading_error

            if w > self.w_max:
                w = self.w_max
            elif w < -self.w_max:
                w = - self.w_max

            self.motion.evaluate_vw(0, w, delta_t)


class AbsoluteRotationProfileControl(PositionControl):

    def __init__(self, motion, _w_accel, _w_decel, _w_max):
        super(AbsoluteRotationProfileControl, self).__init__(motion)
        self.w_accel = _w_accel
        self.w_decel = _w_decel
        self.w_max = _w_max
        self.decel_angle = self.w_max * self.w_max / (2.0 * self.w_decel)
        self.threshold = math.radians(1)
        self.target_w = 0

    def set_target(self, target_heading):
        self.reset()
        self.target_heading = target_heading
        self.target_w = 0

    def evaluate(self, delta_t):
        heading_error = normalize_angle_radians(math.radians(self.target_heading) - self.motion.theta)

        if (abs(heading_error) < self.threshold) or (self._target_got):
            self._target_got = True
            self.motion.evaluate_vw(0, 0, delta_t)
        else:
            if heading_error >= 0:
                s = 1
            else:
                s = -1
                heading_error = -heading_error

            (_, current_w) = self.motion.get_speeds_vw()

            if heading_error < self.decel_angle:
                # fase di decelerazione
                expected_w = math.sqrt(self.w_max * self.w_max - 2 * self.w_decel * (self.decel_angle - heading_error))
                if expected_w > abs(current_w):
                    # siamo ancora in fase di accelerazione
                    self.target_w = self.target_w + self.w_accel * delta_t
                    if self.target_w > self.w_max:
                        self.target_w = self.w_max
                else:
                    self.target_w = expected_w
            else:
                # fase di accelerazione o moto a vel costante
                self.target_w = self.target_w + self.w_accel * delta_t
                if self.target_w > self.w_max:
                    self.target_w = self.w_max

            self._target_got = False
            self.motion.evaluate_vw(0, s * self.target_w, delta_t)


class RelativeRotationProportional(PositionControl):

    def __init__(self, motion, _kp, _w_max):
        super(RelativeRotationProportional, self).__init__(motion)
        self.w_max = _w_max
        self.kp = _kp
        self.threshold = math.radians(1)
        self.cumulative_theta = 0
        self.target_rotation = 0

    def set_target(self, target):
        self.reset()
        self.target_rotation = target
        self.cumulative_theta = 0

    def get_target(self):
        return self.target_rotation

    def evaluate(self, delta_t, theta=None):

        if theta is None:
            heading_error = math.radians(self.target_rotation) - self.cumulative_theta
        else:
            heading_error = theta

        if (abs(heading_error) < self.threshold) or self._target_got:
            self._target_got = True
            self.motion.evaluate_vw(0, 0, delta_t)
            self.target_rotation = 0
        else:
            w = self.kp * heading_error
            if w > self.w_max:
                w = self.w_max
            elif w < -self.w_max:
                w = - self.w_max
            self.motion.evaluate_vw(0, w, delta_t)

            self.cumulative_theta = self.cumulative_theta + self.motion.delta_theta
            # print(self.cumulative_theta)
            self._target_got = False


class HeadingToControl(PositionControl):

    def __init__(self, absolute_rotation):
        self.absolute_rotation = absolute_rotation
        super(HeadingToControl, self).__init__(absolute_rotation.motion)

    def set_target(self, x, y):
        self.reset()
        target_heading = math.atan2(y - self.motion.y, x - self.motion.x)
        self.absolute_rotation.set_target(math.degrees(target_heading))

    def evaluate(self, delta_t):
        self.absolute_rotation.evaluate(delta_t)
        self._target_got = self.absolute_rotation._target_got


class AlignControl(PositionControl):
    def __init__(self, motion, sensors, kw, _w_max):
        super(AlignControl, self).__init__(motion)
        self.motion = motion
        self.sensors = sensors
        self.kw = kw
        self.threshold = 0.0005
        self.w_max = _w_max

    def set_target(self):
        self.reset()


    def evaluate(self, delta_t):

        self.sensors.evaluate(delta_t, resolution=45)

        heading_error = self.sensors.memory[315] - self.sensors.memory[45]
        print('heading error:', self.sensors.memory[315], self.sensors.memory[45])
        if (abs(heading_error) < self.threshold) or self._target_got:
            self._target_got = True
            self.motion.evaluate_vw(0, 0, delta_t)
            self.target_rotation = 0
        else:
            w = self.kw * heading_error
            if w > self.w_max:
                w = self.w_max
            elif w < -self.w_max:
                w = - self.w_max

            self.motion.evaluate_vw(0, w, delta_t)
            self._target_got = False
