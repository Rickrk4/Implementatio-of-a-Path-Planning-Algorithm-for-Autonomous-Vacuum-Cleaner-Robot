#
#
#
import math


class ElectricPart:

    def __init__(self, _R, _L):
        self.R = _R
        self.L = _L
        self.I = 0

    def evaluate(self, V, delta_t):
        self.I = (1 - self.R * delta_t / self.L) * self.I + delta_t / self.L * V
        return self.I



class TorqueToOmega:

    def __init__(self, _J, _b):
        self.J = _J
        self.b = _b
        self.w = 0

    def evaluate(self, T, delta_t):
        self.w = (1 - self.b * delta_t / self.J) * self.w + delta_t / self.J * T
        return self.w




class DCMotorWithWheel:

    # _R, resistance of the inductor
    # _L, impedence of the inductor
    # _Kt, torque coefficient
    # _Ke, BackEMF coefficient
    # _gear, Gearbox ratio
    # _Wm, Wheel Mass
    # _Wr, Wheel Radius
    # _M, robot mass part
    # _b, Friction coefficient
    #
    def __init__(self, _R, _L, _Kt, _Ke, _gear, _Wm, _Wr, _M, _b):
        # inertial moment of the wheel
        _J = 0.5 * _Wm * _Wr**2
        # electric part
        self.electric = ElectricPart(_R, _L)
        # motion part
        self.torque_to_omega = TorqueToOmega(_J, _b)
        # other coefficients
        self.Kt =_Kt
        self.Ke = _Ke
        self.wheel_radius = _Wr
        self.mass = _M
        self.b = _b
        self.gear = _gear
        self.omega = 0
        self.v = 0
        self.accel = 0

    def get_speed(self):
        return self.v

    def evaluate(self, Vin, delta_t):
        # electric part
        V = Vin - self.Ke * self.omega
        I = self.electric.evaluate(V, delta_t)
        Tm = I * self.Kt
        # external load computation
        Fload = self.mass * abs(self.accel) + self.b * abs(self.v)
        Tload = Fload * self.wheel_radius
        if Tload < 0:
            Tload = 0
        Tload = Tload / self.gear
        if Tm > 0:
            T = Tm - Tload
            if T < 0:
                T = 0
        else:
            T = Tm + Tload
            if T > 0:
                T = 0
        #print(Tm, Tload, self.accel, T)
        # omega computation
        self.omega = self.torque_to_omega.evaluate(T, delta_t) / self.gear
        v_now = self.omega * self.wheel_radius
        self.accel = (v_now - self.v) / delta_t
        self.v = v_now
        return self.v

    def stop(self):
        self.v = self.omega = self.accel = 0.0
        self.torque_to_omega.w = 0
        self.electric.I = 0



class Motor2642_with_gearbox(DCMotorWithWheel):

    # _Wm, Wheel Mass
    # _Wr, Wheel Radius
    # _M, robot mass part
    # _b, Friction coefficient
    #
    def __init__(self, _Wm, _Wr, _M, _b):
        DCMotorWithWheel.__init__(  self,
                                    1.45,     # R, 1.45 ohm
                                    130e-6,   # L, 130 microHenry
                                    0.0169,   # Kt, torque constant 16.9 mNm/A
                                    (60.0 * 0.00177)/(2*math.pi),  # Ke, back EMF constant, 1.77 mV/rpm
                                    14.0,     # gearbox 14:1
                                    _Wm,      # Wheel Mass
                                    _Wr,      # Wheel Radius, 24mm
                                    _M,       # robot mass part
                                    _b )      # Friction coefficient


