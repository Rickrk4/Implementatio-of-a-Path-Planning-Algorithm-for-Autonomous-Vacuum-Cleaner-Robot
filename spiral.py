#
# path_control
#
from random import randrange
from environment import next_point


class Spiral:
    TAG = 'spiral'

    def __init__(self, r, command_dict, sensors, env):
        self.robot = r
        self.motion = r.motion
        self.command_dict = command_dict
        self.sensors = sensors
        self.w = 15
        self._w = 1 * 1e-4
        self._v = 1 * 1e-5
        self.v = 1
        self.start = False


    def evaluate(self, delta_t):

        if self.sensors[3].evaluate()[0] < self.sensors[3].distance and not self.start:
            return -1
        else:
            self.start = True

        if self.w > 0.01:
            self.w -= self._w
            self.v += self._v

        self.motion.evaluate_vw(self.v, self.w, delta_t)
        #print('current speeds', self.motion.get_speeds_vw())

        print(self.v, self.w)
        if self.sensors[0].evaluate():
            print('collisione rilevata. fine algoritmo')
            self.start = False
            return -1



