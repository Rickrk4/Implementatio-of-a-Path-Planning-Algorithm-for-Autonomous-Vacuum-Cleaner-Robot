#
# telemetry.py
#

import pylab

class SpeedTelemetry:

    def __init__(self, motion):
        self.motion = motion
        self.reset()

    def reset(self):
        self.times = [ ]
        self.target_vl_array = [ ]
        self.target_vr_array = [ ]
        self.current_vl_array = [ ]
        self.current_vr_array = [ ]
        self.current_w_array = [ ]
        self.t = 0

    def gather(self, delta_t):
        self.times.append(self.t)
        self.target_vl_array.append(self.motion.target_vl)
        self.target_vr_array.append(self.motion.target_vr)
        self.current_vl_array.append(self.motion.current_vl)
        self.current_vr_array.append(self.motion.current_vr)
        (v, w) = self.motion.get_speeds_vw()
        self.current_w_array.append(w)
        self.t = self.t + delta_t

    def show(self):
        pylab.figure(1)
        pylab.plot(self.times, self.target_vl_array, 'b-+', label="V Left Target")
        pylab.plot(self.times, self.current_vl_array, 'r-+', label="V Left Current")
        pylab.legend()

        pylab.figure(2)
        pylab.plot(self.times, self.target_vr_array, 'b-+', label="V Right Target")
        pylab.plot(self.times, self.current_vr_array, 'r-+', label="V Right Current")
        pylab.legend()

        pylab.figure(3)
        pylab.plot(self.times, self.current_w_array, 'r-+', label="Omega Current")
        pylab.legend()

        pylab.show()



