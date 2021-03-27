
import math

from controllers import *
from dc_motor import *

class MotionSystem:

    def __init__(self, _M, _wheelbase, _wheel_mass, _wheel_radius,  _kp_speed, _ki_speed, x=0, y=0, theta = 0):
        friction = 7e-5
        self.__left_w = Motor2642_with_gearbox( _wheel_mass,
                                                _wheel_radius,
                                                _M / 2.0,
                                                friction )

        self.__right_w = Motor2642_with_gearbox( _wheel_mass,
                                                _wheel_radius,
                                                _M / 2.0,
                                                friction )

        self.wheelbase = _wheelbase
        self.x = x
        self.y = y
        self.theta = theta
        # controllori velocita'
        self.__speed_pi_left = PI_SAT_Controller(_kp_speed, _ki_speed, 12)
        self.__speed_pi_right = PI_SAT_Controller(_kp_speed, _ki_speed, 12)
        # velocita' attuali
        self.current_vl = 0
        self.current_vr = 0
        self.target_vl = 0
        self.target_vr = 0
        self.threshold = 0.005

        self.last_target_v = self.last_target_w = 0


    def evaluate(self, vl, vr, delta_t):

        #print('Motion.evaluate(vl=', vl, ", vr=", vr, ', delta_t=', delta_t, ')')

        self.target_vl = vl
        self.target_vr = vr
        # aggiornamento odometria
        delta_L = self.__left_w.get_speed() * delta_t
        delta_R = self.__right_w.get_speed() * delta_t
        self.delta_theta = (delta_R - delta_L) / self.wheelbase
        self.delta_linear = (delta_L + delta_R) / 2.0

        self.x = self.x + self.delta_linear * math.cos(self.theta + self.delta_theta / 2.0)
        self.y = self.y + self.delta_linear * math.sin(self.theta + self.delta_theta / 2.0)
        self.theta = self.theta + self.delta_theta

        if self.theta > math.pi:
            self.theta = self.theta - 2*math.pi
        if self.theta < -math.pi:
            self.theta = 2*math.pi + self.theta

        # calcolo controllori di velocita' ruote
        self.current_vl = self.__left_w.get_speed()
        self.current_vr = self.__right_w.get_speed()
        output_L = self.__speed_pi_left.evaluate(self.target_vl - self.current_vl, delta_t)
        output_R = self.__speed_pi_right.evaluate(self.target_vr - self.current_vr , delta_t)

        # applicazione dell'output ai motori
        self.__left_w.evaluate(output_L, delta_t)
        self.__right_w.evaluate(output_R, delta_t)

    def evaluate_vw(self, v, w, delta_t, log = False):
        #print('vel', v, w)
        vl = v - (w * self.wheelbase / 2)
        vr = v + (w * self.wheelbase / 2)

        self.last_target_v = v
        self.last_target_w = w

        return self.evaluate(vl, vr, delta_t)

    def get_speeds(self):
        return (self.__left_w.get_speed(), self.__right_w.get_speed())

    def get_speeds_vw(self):
        (vl, vr) =  (self.__left_w.get_speed(), self.__right_w.get_speed())
        return ( (vl+vr)/2.0, (vr - vl)/self.wheelbase )

    def get_pose(self):
        return (self.x, self.y, self.theta)

    def is_stopped(self):
        #Non potrà mai arrivare a 0. Dobbiamo dire che il robot è fermo se le velocità sono minori di un certo limite.
        return self.get_speeds_vw() < (self.threshold, self.threshold) and self.get_speeds_vw()[0] > -self.threshold

    def get_wheel_speeds(self):
        return self.__left_w.get_speed(), self.__right_w.get_speed()
    def stop(self):
        self.__right_w.stop()
        self.__left_w.stop()
        self.current_vl = 0
        self.current_vr = 0

