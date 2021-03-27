#
# geometry.py
#

import math

def normalize_angle_radians(r):
    while r > math.pi:
        r = r - 2 * math.pi
    while r < -math.pi:
        r = r + 2 * math.pi
    return r

