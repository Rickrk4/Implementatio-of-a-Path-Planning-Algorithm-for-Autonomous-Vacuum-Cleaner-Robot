from enum import Enum
from environment import Dirs

import math
import numpy as np


class Sensor:
    def __init__(self, motion, env, dir, distance = 0.1):
        self.motion = motion
        self.env = env
        self.distance = distance
        self.dir = dir

    def evaluate(self):
        (x, y, theta) = self.motion.get_pose()

        theta = theta * 60
        if theta < 0:
            theta = theta + 360
        # print(self.env.print_dir(self.env.degree_to_dir(theta)))

        return self.env.tof_sensor(self.distance, self.dir, False)


class DSensor:
    def __init__(self, motion, env, pose, distance = 0.1):
        self.motion = motion
        self.env = env
        self.distance = distance
        self.x = pose[0]
        self.y = pose[1]
        self.theta = np.radians(pose[2])

    def evaluate(self):
        current_pose = (self.x * math.cos(self.theta), self.x * math.sin(self.theta), self.theta)
        return self.env.d_sensor(self.distance, (self.x, self.y, self.theta))


class TidalSensor:
    def __init__(self, motion, env, pose, distance = 0.1):
        self.motion = motion
        self.env = env
        self.distance = distance
        self.x = pose[0]
        self.y = pose[1]
        self.theta = np.radians(pose[2])

        self.memory = np.zeros((360))
        self.current_theta = 0

        self.min = self.distance
        self.alfa = 0

    def snapshot(self, px, py, r):
        # Some magick for processing memory array and get a matrix
        return self.env.snapshot(px, py, r)

    def evaluate(self, absolute=False, resolution=1000, r=1):
        min = self.distance
        alfa = 0
        # Un po pesante
        for i in range(0, 360, r):
            # print(i)
            m = self.env.d_sensor(self.distance, (self.x, self.y, i), resolution=resolution,
                                  absolute=absolute)
            # print(i, m)
            # todo: inserire rumore

            # da testare
            self.memory[i] = m

            if m < min:
                min = m
                alfa = i
        return min, alfa


class IRSensor:
    def __init__(self, e, theta, offset, range=0.005):
        self.theta = theta
        self.offset = offset
        self.range = range
        self.e = e

    def evaluate(self):
        return self.e.IRsensor(self.theta, self.offset, self.range)


class BumpSensor:
    def __init__(self, env, motion):
        self.env = env
        self.motion = motion
        self.c = 10

    def evaluate(self):
        return self.env.bump
        '''
        x, y, theta = self.motion.get_pose()
        return self.env.collision_detection(x, y)
        ## Abbiamo 10 evaluate per allontanarci dalla collisione
        if self.c == 0:
            self.c = 100
            x, y, theta = self.motion.get_pose()
            return self.env.collision_detection(x,y)
        self.c -= 1
        print('vai')
        return False
        '''
