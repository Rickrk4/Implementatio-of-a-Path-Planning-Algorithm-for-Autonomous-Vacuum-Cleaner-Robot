import sys

from GA_path_control import GA_path_control
from motion import *
from random_path_control import RandomPathControl
from rotation_control import *
from polar_control import *
from path_control import *
#from dwa_control import *
from PPCR_path_control import *
from telemetry import *
from test_sensor import *
from dummy_control import DummyControl
from environment import Dirs
from random_walk import RandomWalk
from zig_zag import ZigZag
from boundary_walk import BoundaryWalk
from spiral import Spiral
from smooth_vw_control import SmoothVWControl
from MemoryMap import MemoryMap

from rotation_control import AlignControl


def load_array(file, delimiter=','):
    text_file = open(file, "r")
    lines = text_file.read().replace('\n', '')[2:-2].split(']' + delimiter + ' [')
    l = list(map( lambda str:  list(map(lambda x : int(x), str.split(delimiter))), lines))
    return l


def array_to_str(a):
    return ''.join(str(x) for x in a)


def compress_array(a, px, py, theta, invert_y_axis=False):
    _y = -1 if invert_y_axis else 1

    dirs = {
        "01": 90,
        "11": 45,
        "10": 0,
        "1-1": 315,
        "0-1": 270,
        "-1-1": 225,
        "-10": 180,
        "-11": 135
    }

    current_theta = theta
    current_position = [px, py]
    r = []
    p = a[0]
    r.append(('rotate', [dirs[array_to_str(p)]]))
    c = 1

    for x in a[1:]:
        if (p == x):
            c = c + 1
        else:
            current_position = [current_position[0] + p[0] * c * _y, current_position[1] + p[1] * c]
            r.append(('go_to',
                      [round(0.5 - current_position[0] / 10.0 - 0.05, 2), round(current_position[1] / 10.0 + 0.05, 2)]))
            theta = dirs[array_to_str(x)]
            if theta < -180:
                theta += 360
            if theta > +180:
                theta -= 360
            r.append(('rotate', [theta]))
            p = x
            c = 1
    current_position = [current_position[0] + p[0] * c * _y, current_position[1] + p[1] * c]
    r.append(('go_to', [round(0.5 - current_position[0] / 10.0 - 0.05, 2), round(current_position[1] / 10.0 + 0.05, 2)]))

    #r.append(('go_to', [round(0.5 - p[0] / 10.0 - 0.05, 2), round(p[1] / 10.0 + 0.05, 2)]))
    # r.append(('go_to', [p[1]*c, p[0]*c]))
    return r


class Robot:

    def __init__(self, e, diameter):
        self.e = e
        self.d = diameter
        self.motion = MotionSystem(15,  # robot mass, 15kg
                                   0.33,  # wheelbase, 330mm
                                   0.08,  # wheel_mass, 80 gr
                                   0.024,  # wheel_radius, 24mm
                                   400,  # kp speed
                                   300,  # ki speed
                                   x= 0.05, y=0.05, theta= np.radians(90)) #120

        self.sensor = [BumpSensor(e, self.motion), None, None, TidalSensor(0.1, e, (0, 0, 0))]#[[Sensor(self.motion, self.e, Dirs.Up), Sensor(self.motion, self.e, Dirs.Right)]

        # self.absolute_rotation = AbsoluteRotationProportionalControl(self.motion,
        #                                                 4,  # kp
        #                                                 6)  # 6 rad/s max

        self.absolute_rotation = AbsoluteRotationProfileControl(self.motion,
                                                                12,  # accel 12 rad/s^2
                                                                12,  # decel 12 rad/s^2
                                                                3)  # 6 rad/s max

        self.relative_rotation = RelativeRotationProportional(self.motion,
                                                              10,  # 10 rad/s max
                                                              20)  # kp = 20

        self.heading_to = HeadingToControl(self.absolute_rotation)

        self.polar = PolarControl(self.motion,
                                  5.5, 1,  # 1 m/s max
                                  # 4, 6,  # 6 rad/s max  #non bastano
                                  10, 2,  # 10 rad/s max #modifichiamo
                                  0.005)   # 5 mm

        self.dummy = DummyControl(self.motion,
                                  5.5, 1,  # 1 m/s max
                                  # self.sensor[0],
                                  #DSensor(0.1, e, (0.05, 0, 0)),
                                  DSensor(self.d, e, (self.d/2, 0, 0)),
                                  0.005)  # 5 mm
        '''
        self.dwa = DWAControl(self.motion,
                              5.5, 1,  # 1 m/s max
                              10, 20,  # 10 rad/s max
                              # self.motion, 5.5, 1, 10, np.radians(10),
                              1e-3, r=0.05, threshold=0.01)
        '''
        self.smooth_vw_control = SmoothVWControl(self.motion,
                                                 5.5, 1,  # 1 m/s max
                                                 # 10, 20,  # 10 rad/s max
                                                 # self.motion, 5.5, 1, 10, np.radians(10),
                                                 None, threshold=0.01)

        self.path_control = PathControl(self.motion,
                                        {'rotate': self.absolute_rotation,
                                         'rotate_relative': self.relative_rotation,
                                         'heading_to': self.heading_to,
                                         'go_to': self.polar,
                                         'go': self.dummy,
                                         #'dwa': self.dwa
                                         })
        self.ga_path_control = GA_path_control(
            "output.txt",
            self.motion,
            {'rotate': self.absolute_rotation,
             'rotate_relative': self.relative_rotation,
             'heading_to': self.heading_to,
             'go_to': self.polar,
             'go': self.dummy,
             #'dwa': self.dwa
             }
        )
        self.random_walk = RandomWalk(self,
                                      {'rotate': self.absolute_rotation,
                                       'rotate_relative': self.relative_rotation,
                                       'heading_to': self.heading_to,
                                       'go_to': self.polar,
                                       'go': self.dummy,
                                       'smooth': self.smooth_vw_control
                                       }, self.sensor, self.e)

        self.zig_zag = ZigZag(self,
                              {'rotate': self.absolute_rotation,
                               'rotate_relative': self.relative_rotation,
                               'heading_to': self.heading_to,
                               'go_to': self.polar,
                               'go': self.dummy,
                               }, self.sensor, self.e)
        self.boundary_walk = BoundaryWalk(self,
                                          {'rotate': self.absolute_rotation,
                                           'rotate_relative': self.relative_rotation,
                                           'heading_to': self.heading_to,
                                           'go_to': self.polar,
                                           'go': self.dummy,
                                           }, self.sensor, self.e)
        self.spiral = Spiral(self,
                             {'rotate': self.absolute_rotation,
                              'rotate_relative': self.relative_rotation,
                              'heading_to': self.heading_to,
                              'go_to': self.polar,
                              'go': self.dummy,
                              },self.sensor, self.e)

        self.random_path_control = RandomPathControl([
            [self.boundary_walk, 20],
            [self.zig_zag, 20],
            [self.random_walk, 30],
            [self.spiral, 30]
        ])

        self.alignControl = AlignControl(self.motion, TidalSensor(0.1, e, (0, 0, 0)), 100, 10)
        self.bump = BumpSensor(e, self.motion)
        # heading_to sembra introdurre un erroe di convergenza.
        # meglio usare rotazioni relative
        # self.path_control.add( ( 'heading_to', [0, 1] ) )

        a = load_array('output.txt', delimiter=',')
        r = compress_array(a, 4, 0, 0, True)

        for cmd in r:
            self.path_control.add(cmd)

        #self.path_control.add(('rotate_relative', [0]))
        #self.path_control.add(('go_to', [0.05, 0.45]))
        #self.path_control.add(('rotate', [0]))
        #self.path_control.add(('go_to', [0.35, 0.45]))

        # self.path_control.add(('dwa', [0.3, 0.3]))
        # self.path_control.add(('dwa', [0.15, 0.15]))
        # self.path_control.add(('dwa', [0.05, 0.4]))
        # self.path_control.add(('go_to', [0, 0.2]))
        # self.path_control.add(('go_to', [0, 0.3]))
        # self.path_control.add(('go_to', [0, 0.4]))
        # self.path_control.add(('go_to', [0, 0.5]))
        # self.path_control.add(('go_to', [0.4, 0.2]))
        # self.path_control.add(('heading_to', [0.2, 0.2]))
        # self.path_control.add(('rotate_relative', [-20]))
        # self.path_control.add(('rotate_relative', [+40]))
        # self.path_control.add(('rotate_relative', [-20]))

        self.telemetry = SpeedTelemetry(self.motion)
        self.map = MemoryMap(e, self.sensor[3])

        #self.start_x, self.start_y, _ = self.get_pose()

        self.scan_timer = self.scan_timer_time = 1000 #1 sec

        self.state = 0;

    def get_wheel_speeds(self):
        return self.motion.get_wheel_speeds()

    def get_pose(self):
        return self.motion.get_pose()

    def get_diameter(self):
        return self.d

    def evaluate(self, delta_t):
        #self.e.evaluate(delta_t)
        # self.sensor[0].evaluate()
        # self.polar.evaluate(0.45, 0.25)
        # self.absolute_rotation.evaluate(90, delta_t)
        # self.relative_rotation.evaluate(90, delta_t)

        # I Vari approcci nuovi
        #self.random_walk.evaluate(delta_t)
        #self.zig_zag.evaluate(delta_t)
        #self.boundary_walk.evaluate(delta_t)
        #self.spiral.evaluate(delta_t)
        # print(self.dummy.target_got())

        #self.dummy.evaluate(delta_t)
        #self.motion.evaluate(vl= 0.55 , vr= 0.55 , delta_t = 0.001 )
        # self.motion.evaluate_vw(0, 10, delta_t)
        self.e.evaluate(delta_t)
        #self.path_control.evaluate(delta_t)

        self.ga_path_control.evaluate(delta_t)
        #self.random_path_control.evaluate(delta_t)

        # print(self.motion.get_pose())
        # self.motion.evaluate(.5, .5, delta_t)
        self.telemetry.gather(delta_t)

        #_, _, theta = self.get_pose()
        x, y, __ = self.e.get_discrete_position()

        if self.scan_timer == 0:
            _x, _y, _ = self.get_pose()
            x = int(_x / self.d)
            y = int(_y / self.d)
            self.map.evaluate(x, y, None)
            # print(self.map.get(),'\n')
            #self.x = x
            #self.y = y

            print(self.map.M[self.map.min_x+1:self.map.max_x,self.map.min_y+1:self.map.max_y])
            self.scan_timer = float(str(delta_t)[::-1])  #self.scan_timer_time
        else:
            self.scan_timer -= 1

        if self.telemetry.t > 120 and self.random_path_control.end:
            self.map.save("map.txt")
            #self.telemetry.show()
            #print(self.e.coverage())
            sys.exit(0)

## Todo: implementare align anche in boundary walk.
## TODO: combinare i vari approcci.
## TODO: aggioustare il salvataggio della memorymap.