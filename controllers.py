# controllers.py

class P_Controller:

    def __init__(self, kp):
        self.kp = kp

    def evaluate(self, u):
        return self.kp*u


class PSAT_Controller:

    def __init__(self, kp, _max):
        self.kp = kp
        self._max = _max

    def evaluate(self, u):
        output = self.kp*u
        if output > self._max:
            output = self._max
        elif output < - self._max:
            output = -self._max
        return output


class PI_Controller:

    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral_term = 0

    def evaluate(self, u, delta_t):
        self.integral_term = self.integral_term + u * delta_t
        return self.kp * u + self.ki * self.integral_term


class PI_SAT_Controller:

    def __init__(self, kp, ki, sat):
        self.kp = kp
        self.ki = ki
        self.saturation = sat
        self.integral_term = 0
        self.saturation_flag = False

    def evaluate(self, u, delta_t):
        if not(self.saturation_flag):
            self.integral_term = self.integral_term + u * delta_t

        output = self.kp * u + self.ki * self.integral_term

        if output > self.saturation:
            self.saturation_flag = True
            output = self.saturation
        elif output < - self.saturation:
            self.saturation_flag = True
            output = -self.saturation
        else:
            self.saturation_flag = False
        return output


