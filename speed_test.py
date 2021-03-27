#
#
#

import math
import pylab

from mobile_robot import *

wheel_mass = 0.08 # 80 gr
wheel_radius = 0.024 # 24 mm
robot_mass = 15 # 15 kg total
friction = 7e-5

robot = Robot(  robot_mass,
                0.33,   # wheelbase
                wheel_mass,
                wheel_radius,
                80,
                120)

v_target_max = 0.5 # m/s
t = 0
delta_t = 1e-4 # 0.1ms

v_target = 0
accel = 1.3 # m/s2

times = []
v_left_array = [ ]
v_right_array = [ ]

v_current = 0

while t < 2:

    robot.evaluate(v_target, -v_target*0.5, delta_t)

    t = t + delta_t

    s = robot.get_speeds()

    v_left_array.append(s[0])
    v_right_array.append(s[1])
    times.append(t)

    v_target = v_target + accel * delta_t
    if v_target > v_target_max:
        v_target = v_target_max


pylab.figure(1)
pylab.plot(times, v_left_array, 'b-+', label="V Left")
pylab.plot(times, v_right_array, 'r-+', label="V Right")
pylab.legend()

pylab.show()

