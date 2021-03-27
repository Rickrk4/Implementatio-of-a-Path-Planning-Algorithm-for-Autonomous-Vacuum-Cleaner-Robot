#
# path_control
#

import numpy as np
from environment import Dirs
import math


def add_col(A):
    return np.hstack([A, np.hsplit(np.zeros(A.shape[0]), A.shape[0])])


def add_row(A):
    return np.vstack([A, np.zeros(A.shape[1])])


def add_n_row(A, n):
    for i in range(0, n):
        A = add_row(A)
    return A


def add_n_col(A, n):
    for i in range(0, n):
        A = add_col(A)
    return A


class PPCR_PathControl:
    TAG = 'PPCR_PathControl'
    n_dirs = {Dirs.Left: [-1, 0], Dirs.Right: [1, 0], Dirs.Up: [0, 1], Dirs.Down: [0, -1]}  # y axis is inverted
    dirs = {Dirs.Left: 180, Dirs.Right: 0, Dirs.Up: 90, Dirs.Down: 270}

    def __init__(self, r, command_dict, sensors, env):
        self.robot = r
        self.motion = r.motion
        self.command_dict = command_dict
        self.sensors = sensors
        self.running = False
        self.control_time = 0.005
        self.e = np.zeros([0, 0])
        self.env = env
        self.dir = {Dirs.Left: np.array([-1, 0]), Dirs.Right: np.array([1, 0]), Dirs.Up: np.array([0, 1]),
                    Dirs.Down: np.array([0, -1])};
        self.lock = False
        self.state = 'go_to'
        self.last_pose = self.packed_continuous2discrete_position(self.motion.get_pose())
        self.test = 0
    def add(self, command):
        self.command_list.append(command)

    def adjust_matrix(self, A, x, y):
        if x > A.shape[0]:
            A = add_n_row(A, x - A.shape[0])
        if y > A.shape[1]:
            A = add_n_col(A, y - A.shape[1])
        return A

    def degree_to_dir(self, degree):
        floor = int(degree / 90)
        sealing = floor + 1
        if abs(degree - floor * 90) <= abs(degree - sealing * 90):
            return Dirs((floor % 4) * 90)
        else:
            return Dirs((sealing % 4) * 90)

    def continuous2discrete_position(self, x, y, theta=0, d=0):
        if d == 0:
            d = self.robot.d

        x = int(x / d)
        y = int(y / d)
        theta = theta * 60
        if theta < 0:
            theta = theta + 360

        _dir = self.degree_to_dir(theta)
        return x, y, _dir

    # facade for provide t-uple to continuas.. method, cause python not support methond overloading
    def packed_continuous2discrete_position(self, t, _d=0):
        return self.continuous2discrete_position(t[0], t[1], t[2], d=_d)

    def change_state(self, state):
        self.state = state
        self.command_dict[self.state].reset()
        self.running = False

    def evaluate(self, delta_t):



        new_pose = self.packed_continuous2discrete_position(self.motion.get_pose(), 0.02)
        x, y, _ = self.packed_continuous2discrete_position(self.motion.get_pose())
        self.e = self.adjust_matrix(self.e, x + 1, y + 1)
        self.e[x][y] = 1


        if (new_pose == self.last_pose or self.lock) and not self.command_dict[
            self.state].target_got() and self.running:
            self.command_dict[self.state].evaluate(delta_t)
            return

        if self.last_pose != new_pose:
            self.last_pose = new_pose

        d = self.sensors[0].evaluate()
        print('d=', d)

        if self.state == 'rotate_relative':

            if self.command_dict['rotate_relative'].target_got() or not self.running:
                _x, _y = np.sum([[x, y], self.dir[_]], 0)

                if d <= 0.005 or (_x < self.e.shape[0] and _y < self.e.shape[1] and self.e[_x][_y] == 1):
                    self.command_dict['rotate_relative'].set_target(+90)
                    self.running = True
                else:

                    (x, y, theta) = self.motion.get_pose()
                    (_x, _y, _dir) = self.packed_continuous2discrete_position(self.motion.get_pose())
                    _x = int(x * 10)
                    _y = int(y * 10)
                    sensor_orientation = self.sensors[1].dir
                    relative_dir = Dirs(max(self.dirs[_dir] - 90 + self.dirs[sensor_orientation],
                                            (self.dirs[_dir] + self.dirs[sensor_orientation] - 90 + 360) % 360))
                    c = np.sum([[_x, _y], self.n_dirs[relative_dir]], 0)

                    if self.test >= 2:
                        print(self.test, x,y,   self.sensors[1].evaluate(), c, self.e[c[0]][c[1]])

                    if self.sensors[1].evaluate() > self.robot.d/2 and self.e[c[0]][c[1]] == 0.0:
                        self.command_dict['rotate_relative'].set_target(-90)
                        self.running = True
                    else:
                        self.change_state('go_to')
            else:

                self.command_dict[self.state].evaluate(delta_t)

        (x, y, theta) = self.motion.get_pose()
        (_x, _y, _dir) = self.packed_continuous2discrete_position(self.motion.get_pose())
        _x = int(x * 10)
        _y = int(y * 10)
        sensor_orientation = self.sensors[1].dir
        relative_dir = Dirs( max(self.dirs[_dir]-90+self.dirs[sensor_orientation], (self.dirs[_dir]+self.dirs[sensor_orientation]-90+360)%360))
        c = np.sum([[_x, _y], self.n_dirs[ relative_dir ]], 0)

        if self.sensors[1].evaluate() > self.robot.d/2 and self.e[c[0]][c[1]] == 0:

            self.lock = True
            self.test = self.test +1

        if self.state == 'go_to':

            if self.command_dict['go_to'].target_got():
                # print('state change to rotation')
                self.change_state('rotate_relative')
                self.lock = False
            else:
                if not self.lock:
                    x_0, y_0, theta = self.motion.get_pose()
                    _x_0, _y_0, dir = self.continuous2discrete_position(x_0, y_0, theta)

                    x, y = np.sum([[x_0, y_0], self.dir[dir] * d], 0)
                    _x, _y = np.sum([[_x_0, _y_0], self.dir[dir]], 0)

                    if _x < self.e.shape[0] and _y < self.e.shape[1] and self.e[_x][_y] == 1:
                        # Se la cella successiva è già stata pulita puntiamo direttamente al centro di quella corrente.
                        self.lock = True
                        # x, y, dir = self.packed_continuous2discrete_position(self.motion.get_pose())
                        if dir == Dirs.Left or dir == Dirs.Right:
                            x = _x_0 / 10 + self.robot.d / 2
                            y = y_0
                        else:
                            x = x_0
                            y = _y_0 / 10 + self.robot.d / 2

                    self.command_dict['go_to'].set_target(x, y)
                    self.running = True

                self.command_dict[self.state].evaluate(delta_t)
