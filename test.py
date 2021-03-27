from environment import Environment

import numpy as np
import math


class Robot:
    def get_pose(self):
        return (0.05, 0.0, np.radians(180))

class Test:

    def __init__(self, r):
        self.r = r
        self.n = 5
        self.m = 5

    def out_of_bound(self, x, y):
        return x < 0 or x >= self.n or y < 0 or y >= self.m

    def next_point(self, x, y, theta, d):
        _x = x + d * math.cos(theta)
        _y = y + d * math.sin(theta)
        return float("{:.2f}".format(_x)), float("{:.2f}".format(_y))

    def d_sensor(self, d, resolution=10):
        x, y, theta = self.r.get_pose()
        for i in np.arange(d, 0.0, -d / resolution):
            _x, _y = self.next_point(x, y, theta, i)
            if not self.out_of_bound(_x*10, _y*10):
                return i
        return 0.0


r = Robot()
e = Test(r)

print(e.d_sensor(0.1))

A = np.zeros((5,5))

with open("ciao.txt","w") as o:
    o.write(  "(" + str(5) +", " + str(5) + ")\n")
    for x in A:
        o.write(str(x) + "\n")