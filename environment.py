import numpy as np
from enum import Enum
from random import random
import math


class Dirs(Enum):
    Up = 90
    Right = 0
    Down = 270
    Left = 180


def next_point(x, y, theta, d):
    # print('cos di ',theta,math.cos(theta))
    #print('calcoliamo il punto a distanza %s da (%s,%s)', d, x, y)
    _x = x + d * math.cos(theta)
    _y = y + d * math.sin(theta)
    return float("{:.2f}".format(_x)), float("{:.2f}".format(_y))


class Environment:
    TAG = 'Env'

    dirs = {Dirs.Left: 180, Dirs.Right: 0, Dirs.Up: 90, Dirs.Down: 270}
    n_dirs = {Dirs.Left: [-1, 0], Dirs.Right: [1, 0], Dirs.Up: [0, 1], Dirs.Down: [0, -1]}  # y axis is inverted
    dirs_euclidean_distance = {Dirs.Left: -1, Dirs.Right: 1, Dirs.Up: 1, Dirs.Down: -1}  # y axis is inverted
    print_dir = {Dirs.Left: 'Left', Dirs.Right: 'Right', Dirs.Up: 'Up', Dirs.Down: 'Down'}

    def __init__(self, n, m):
        self.n = n
        self.m = m
        self.e = np.zeros((self.n, self.m))
        self.e[2][2] = 2
        #self.e[0][0] = 2
        self.e[4][4] = 2

        print(self.e)

        self.bump = False
        self.l_bump = False
        self.r_bump = False
        self.t = 0
        self.counter = 0
        self.message_timer = 0  # 2 msg per second
        self.obstacle = self.get_obstcle_position()
        self.boh = 0
    def set_robot(self, r):
        self.r = r
        self.pose = self.r.get_pose()

    def get_obstcle_position(self):
        obstacles = []
        for i in range(0, self.n):
            for j in range(0, self.m):
                if self.e[i][j] == 2:
                    obstacles.append((i, j))
        return obstacles

    def degree_to_dir(self, degree):
        floor = int(degree / 90)
        sealing = floor + 1
        if abs(degree - floor * 90) <= abs(degree - sealing * 90):
            return Dirs((floor % 4) * 90)
        else:
            return Dirs((sealing % 4) * 90)

    def get_discrete_position(self):

        z = 10 * 0.1 / self.r.get_diameter()

        (x, y, theta) = self.r.get_pose()
        d = self.r.get_diameter()
        x = round(x * z)#round(x / d)
        y = round(y * z)#round(y / d)
        theta = theta * 60
        if theta < 0:
            theta = theta + 360
        _dir = self.degree_to_dir(theta)
        return int(x), int(y), _dir

    def _get_discrete_position(self, x, y, theta):
        z = 10 * 0.1 / self.r.get_diameter()

        d = self.r.get_diameter()
        x = round(x * z)#round(x / d)
        y = round(y * z)#round(y / d)
        theta = theta * 60
        if theta < 0:
            theta = theta + 360
        _dir = self.degree_to_dir(theta)
        return int(x), int(y), _dir

    def out_of_bound(self, x, y):
        return x < 0 or x >= self.n or y < 0 or y >= self.m

    ## higher resolution => higher precision
    def d_sensor(self, d, sensor_pose, resolution=10, absolute=False):

        x, y, theta = self.r.get_pose()

        theta += np.radians(sensor_pose[2])
        if absolute:
            theta = np.radians(sensor_pose[2])#sensor_pose[2]
        x += sensor_pose[0] * math.cos(theta)
        y += sensor_pose[0] * math.sin(theta)
        for i in np.arange(d, 0.0, -d / resolution):
            _x, _y = next_point(x, y, theta, i)
            # print(_x, _y, sensor_pose)
            # if not self.out_of_bound(_x * 10, _y * 10) and self.e[int(_x * 10)][int(_y * 10)] != 2:
            __x = _x / self.r.get_diameter()
            __y = _y / self.r.get_diameter()
            if not self.out_of_bound(__x, __y) and self.e[int(__x)][int(__y)] != 2:
                return i
        return 0.005

    '''
    def IRsensor(self, _theta, offset, range):
        x, y, theta = self.r.get_pose()
        x += offset[0]
        y += offset[1]

    
    def _tof_sensor(self, x, y, theta, range):

        d = self.d_sensor(range, (x, y, theta))

        # print(x, y, theta,d)
        return d

    def tof_sensor(self, range, sensor_orientation, log=False):

        (x, y, theta) = self.r.get_pose()
        (_x, _y, _dir) = self.get_discrete_position()
        _x = int(x * 10)
        _y = int(y * 10)

        # if log:
        #    _dir = Dirs( max(self.dirs[_dir]-90+self.dirs[sensor_orientation], (self.dirs[_dir]+self.dirs[sensor_orientation]-90+360)%360))

        c = np.sum([[_x, _y], self.n_dirs[_dir]], 0)

        if log:
            print('cella corrente', (_x, _y), 'prossima cella', c)

        if self.out_of_bound(c[0], c[1]):
            return 0.0
            dy = _y * self.r.get_diameter() + self.r.get_diameter() / 2 - y
            dx = _x * self.r.get_diameter() + self.r.get_diameter() / 2 - x
            return math.hypot(dx, dy)

        return range

        # TODO: rilevare ostacoli

        if (c[1] < self.m or _dir != Dirs.Up) and (c[1] > 0 or _dir != Dirs.Down) and (
                c[0] < self.n or _dir != Dirs.Right) and (c[0] > 0 or _dir != Dirs.Left) and (
                (not (self.m > c[1] >= 0 and 0 <= c[0] < self.n) or (
                        self.m > c[1] >= 0 and 0 <= c[0] < self.n and self.e[c[0]][c[1]] != 2))
        ):
            d = range
        else:

            # print('mi fermo',_x)

            # In y=0.449999 risulta essere in cella 4, ma in 0.45 risulta essere in cella 5,
            # e dunque non si ferma perchè la cella successiva di 5 è 6, e non è stata raggiunta.
            # La soluzione è prendere il minimo tra la prossima cella prevista (al peggio 6) e il
            # limite dello spazio discretizzato, cioè n o m.
            # print("target:",min(c[0], self.n) * self.r.get_diameter()-self.r.get_diameter()/2,min(c[1], self.m) * self.r.get_diameter()+self.r.get_diameter()/2)

            # Attenzione: bug nel riconoscere le celle occupate.
            # Ci si dovrebbe fermare nella cella immediatamente precedente a quella occupata.
            # Il problema è che con il corrente metodo di discretizzazione delle coordinate se
            # si supera, anche di un infitesimo, il centro di una cella le coordinate discretizzare
            # saranno quelle della cella successiva, quindi quella occupata, e la coordinate
            # discretizzate della cella successiva diventeranno quella della cella successiva a quella realmente
            # successiva, che qualora fosse libera farebbero ripartire il robot passando sopra la cella occupata.
            # Sostiture il metodo per discretizzare le coordinate con int(c*10), che fa riferimento al centro del
            # robot, quindi finchè il centro del robot non si trovera in una cella, le coordinate discretizzare
            # saranno quella della cella precedente.

            if (self.m > c[1] >= 0 and 0 <= c[0] < self.n and self.e[c[0]][c[1]] == 2):
                dy = _y * self.r.get_diameter() + self.r.get_diameter() / 2 - y
                dx = _x * self.r.get_diameter() + self.r.get_diameter() / 2 - x

            else:
                dy = min(max(0, c[1]), self.m - 1) * self.r.get_diameter() + self.r.get_diameter() / 2 - y
                dx = min(max(0, c[0]), self.n - 1) * self.r.get_diameter() + self.r.get_diameter() / 2 - x
            d = min(math.hypot(dx, dy), range)

        return d
    '''

    def collision_detection(self, x, y):
        theta = math.degrees(self.r.get_pose()[2])
        if theta < 0:
            theta += 360
        # print(theta)

        for i in range(0, 360, 10):
            _x, _y = next_point(x, y, np.radians(i), self.r.get_diameter()/2 - self.r.get_diameter() * 1e-1)
            __x = _x / self.r.get_diameter()
            __y = _y / self.r.get_diameter()
            #print(__x, __y)
            if self.out_of_bound(__x, __y) or self.e[int(__x), int(__y)] == 2:
                #print('collisione con', (__x, __y), (x,y))
                if theta - 5 > i:
                    self.r_bump = True
                if theta < i - 5:
                    self.l_bumo = True
                return True

        return False

    def collision_detected(self):

        self.r.motion.stop()
        # Da testare
        x, y, _ = self.r.get_pose()
        d = inc = 0.00001  # la precisione della correzione
        while True:
            for i in np.arange(0, 360, 10):
                _x, _y = next_point(x, y, i, d)
                if not self.collision_detection(_x, _y):
                    self.r.motion.x = _x
                    self.r.motion.y = _y
                    return
            d += inc

        # self.r.motion.x = self.pose[0]
        # self.r.motion.y = self.pose[1]
        # print('collisione', random())
        # self.r.motion.stop()

    def coverage(self):
        sum = 0
        for i in range(0, self.n):
            for j in range(0, self.m):
                if self.e[i][j] != 0:
                    sum += 1
        return sum / self.n * self.m

    def evaluate(self, delta_t):
        self.t += delta_t
        self.bump = self.l_bump = self.r_bump = False

        x, y, theta = self.r.get_pose()
        if self.boh > 10 and False:
            exit()
        self.boh = self.boh + 1
        #print('boh', self.boh)

        if self.collision_detection(x, y):
            # il bump rileva solo urti con una certa forza cinetica
            self.bump = self.r.motion.get_speeds_vw()[0] > 0.01  # 0.51 troppo; non rileva i bump
            if self.bump:
                print('Bump!')

            ## Anche se il bump non rileva, il robot è cmq soggetto alle costrizioni fisiche, quindi deve stare fermo.
            ## un approccio migliore potrebbe essere anzichè memorizzare la posa precedente, quando avviane la
            ## collisione cercare nel raggio il punto più vicino al punto attuale privo di collisione.
            ## Potrebbe essere pesante in quanto per valurare la collisinoe di ogni punto facciamo 360 controlli, e per
            ## controllare tutti i punti del raggio quidni dovremmo fare 360*360 controlli. Magari discretizzando il
            ## raggio a 10 gradi, così si fanno 36*360controlli. sempre tanti.
            self.collision_detected()

        else:
            _x = x / self.r.get_diameter()
            _y = y / self.r.get_diameter()
            #self.e[int(_x)][int(_y)] = 1
            #print('nessuna collisione in ', ((x,y),(self.n, self.m)))

        #x, y, _ = self.get_discrete_position()
        #if self.out_of_bound(x, y) == self.e[int(x)][int(y)]:
        #    # if 0 < y < self.m and self.n > x > 0 == self.e[int(x)][int(y)]:
        #    self.e[x][y] = 1

    def snapshot(self, px, py, r):
        A = np.zeros((2 * r + 1, 2 * r + 1))
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if self.out_of_bound(px + i, py + j) or self.e[px + i][py + j] == 2:
                    A[i + 1][j + 1] = 2
        return A
