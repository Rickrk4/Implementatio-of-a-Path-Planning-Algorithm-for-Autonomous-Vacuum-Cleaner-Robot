#
# simple polar control
#

import math
import dwa
import numpy as np
from position_control import *


class VirtualRobot:

    def __init__(self, base, pose=(0.0, 0.0, 0)):
        self.point_cloud = [[0, 0]]
        self.vel = (0.0, 0.0)
        self.pose = pose
        self.goal = None
        self.b = False
        self.config = dwa.Config(
            max_speed=+10.0,
            min_speed=-10.0,

            max_yawrate=np.radians(40.0),

            max_accel=15.0,
            max_dyawrate=np.radians(110.0),

            velocity_resolution=0.1,
            yawrate_resolution=np.radians(1.0),
            dt=0.1,

            predict_time=3.0,
            heading=0.15,
            clearance=0.1,
            velocity=1.0,

            base=[-5, -5, 5, 5])
        self.count = 0

    def set_target(self, x, y):
        self.goal = (x, y)

    def evaluate(self, point_cloud):

        # self.point_cloud = point_cloud

        if self.goal:
            self.vel = dwa.planning(self.pose, self.vel, self.goal,
                                    np.array(self.point_cloud, np.float32), self.config)
            self.pose = dwa.motion(self.pose, self.vel, self.config.dt)

            self.count += 1
            print(self.count, 'virtual robot pose:', self.pose)

            distance = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])

            if distance < 3:
                print('virtual target got')
                self.goal = None

        return self.pose


class DWAControl(PositionControl):

    def __init__(self, motion, kp_lin, v_max, kp_angular, w_max, dt, r, threshold=0.01):
        super(DWAControl, self).__init__(motion)

        self.kp_lin = kp_lin
        self.v_max = v_max
        self.kp_angular = kp_angular
        self.w_max = w_max

        self.motion = motion
        self.threshold = threshold  # 5mm
        self.base = [-r, -r, +r, +r]

        self.counter = 1000

        #self.base = [-0.05, -0.05, +0.05, +0.05]
        self.vr = VirtualRobot((np.array(self.base) * 100).tolist(),
                               (r * 100, r * 100, self.motion.get_pose()[2]))
        # self.vr.set_target(0.2*1000, 0.2*1000)
        #self.point_clouds = [[]]
        # self.get_point_clouds(0.3, 0.0, 1000)
        #self.vr.point_cloud.append([0.3 * 100, 0.05 * 100])
        self.get_point_clouds(0.2,0.2,100)
        '''
        self.vr.point_cloud.append([0.3 * 1000, 0.00 * 1000])
        self.vr.point_cloud.append([0.4 * 1000, 0.00 * 1000])
        self.vr.point_cloud.append([0.3 * 1000, 0.05 * 1000])
        self.vr.point_cloud.append([0.4 * 1000, 0.05 * 1000])
        self.vr.point_cloud.append([0.3 * 1000, 0.10 * 1000])
        self.vr.point_cloud.append([0.4 * 1000, 0.10 * 1000])
        '''

        for i in range(0, 50, 2):
            self.vr.point_cloud.append([i, -1])

        for i in range(0, 50, 2):
            self.vr.point_cloud.append([-1, i])

        for i in range(0, 50, 2):
            self.vr.point_cloud.append([0.5 * 100, i])
            self.vr.point_cloud.append([i, 0.5 * 100])

    def get_point_clouds(self, x, y, resolution=100):
        # Inseriamo solo un punto ogni
        for i in range(0, resolution, int(resolution / 4)):
            self.vr.point_cloud.append([x * resolution + i, y * resolution])
            self.vr.point_cloud.append([x * resolution, y * resolution + i])
            self.vr.point_cloud.append([x * resolution + i, y * resolution + resolution - 1])
            self.vr.point_cloud.append([x * resolution + resolution - 1, y * resolution + i])


    def set_target(self, x, y):
        self.reset()
        self._target_got = False
        self.target_x = x
        self.target_y = y
        self.vr.set_target(x * 100, y * 100)

    def evaluate(self, delta_t, point_cloud=[[0, 0]]):

        self.counter -= 1
        if self.counter == 0:
            self.pose = self.vr.evaluate([[0, 0]])
            self.counter = 100





        dx = self.target_x - self.motion.x
        dy = self.target_y - self.motion.y


        dx = self.vr.pose[0] / 100 - self.motion.x
        dy = self.vr.pose[1] / 100 - self.motion.y
        target_heading = math.atan2(dy, dx)

        real_distance = math.hypot(self.target_x - self.motion.x, self.target_y - self.motion.y)
        distance = math.hypot(dx, dy)


        if not self.vr.goal or real_distance < 0.1:
            # se la distanza è minore di 0.1 vuol dire che siamo nella cella adiacente
            if self.vr.goal:
                print('cambio di controller')
            distance = real_distance
            print('distance=', distance)
            target_heading = math.atan2(self.target_y - self.motion.y, self.target_x - self.motion.x)
            self.vr.goal = None

        if False and distance < real_distance:
            distance = min(distance, real_distance)

        if real_distance <= self.threshold:
            self._target_got = True
            self.motion.evaluate(0.0, 0.0, delta_t)
            print('ho gottato io')
        else:
            self._target_got = False
            heading_error = target_heading - self.motion.theta

            # Mia aggiunta.
            # Quando motion.theta si trova in prossimità del pi cambia di segno repentinamente
            # Questo porta ad avere valori di target heading e motion.theta di valore quasi identico
            # ad avere segno discorde, e dunque a sommarsi al posto di sottrarsi. Un errore prossimo a
            # 0 diventa un errore di circa 2pi. Ho aggiunto quindi il filtro di cui dispone già motion.theta
            # anche all'errore calcolato

            if heading_error > math.pi:
                heading_error = heading_error - 2 * math.pi
            if heading_error < -math.pi:
                heading_error = 2 * math.pi + heading_error

            v = self.kp_lin * distance
            w = self.kp_angular * heading_error

            if v > self.v_max:
                v = self.v_max
            elif v < -self.v_max:
                v = - self.v_max

            if w > self.w_max:
                w = self.w_max
            elif w < -self.w_max:
                w = - self.w_max

            # v = w = 0
            # print('polar control velocities', v, w)
            # self.pose = self.motion.get_pose()
            # self.vel = dwa.planning(self.pose, self.vel, self.goal,
            #                        np.array(point_cloud, np.float32), self.config)

            # self.pose = dwa.motion(self.pose, self.vel, self.config.dt)

            # print('polar velocities', (v,w))

            self.motion.evaluate_vw(v, w, delta_t, False)
