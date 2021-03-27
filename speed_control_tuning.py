#
#
#

import math
import pylab

from dc_motor import *
from controllers import *

wheel_mass = 0.08 # 80 gr
wheel_radius = 0.024 # 24 mm
robot_mass = 15 / 2.0 # 15 kg total, 7.5 on each wheel
friction = 7e-5

motor = Motor2642_with_gearbox(wheel_mass,
                               wheel_radius,
                               robot_mass,
                               friction)


speed_controller = PI_SAT_Controller(80, 120, 12)

v_target_max = 0.5 # m/s
t = 0
delta_t = 1e-4 # 0.1ms

v_target = 0
accel = 1.3 # m/s2

times = []
v_array = [ ]
v_target_array = [ ]

v_current = 0

while t < 2:

    Voltage = speed_controller.evaluate(v_target - v_current, delta_t)
    v_current = motor.evaluate(Voltage, delta_t)

    t = t + delta_t

    v_array.append(v_current)
    v_target_array.append(v_target)
    times.append(t)

    v_target = v_target + accel * delta_t
    if v_target > v_target_max:
        v_target = v_target_max


pylab.figure(1)
pylab.plot(times, v_target_array, 'b-+', label="V Target")
pylab.plot(times, v_array, 'r-+', label="V")
pylab.legend()

pylab.show()

