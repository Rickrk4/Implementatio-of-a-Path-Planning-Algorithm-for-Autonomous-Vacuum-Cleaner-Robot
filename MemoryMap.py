import numpy as np
import math


def next_point(x, y, theta, d):
    # print('cos di ',theta,math.cos(theta))
    _x = x + d * math.cos(theta)
    _y = y + d * math.sin(theta)
    return float("{:.2f}".format(_x)), float("{:.2f}".format(_y))


class MemoryMap:
    R = 512

    def __init__(self, e, sensor):
        self.M = np.zeros((self.R, self.R))
        self.px = self.py = self.min_x = self.min_y = self.max_x = self.max_y = int(self.R / 2)
        self.sensor = sensor
        self.e = e

    def set(self, x, y, value):
        # print(x, y)
        self.M[self.px + x][self.py + y] = value
        if self.px + x < self.min_x:
            self.min_x = self.px + x
        if self.px + x > self.max_x:
            self.max_x = self.px + x
        if self.py + y < self.min_y:
            self.min_y = self.py + y
        if self.py + y > self.max_y:
            self.max_y = self.py + y

    def get(self):
        return self.M[int(self.min_x): int(self.max_x + 1), int(self.min_y): int(self.max_y + 1)]

    def save(self, file_name):
        with open(file_name, "w") as file:
            file.write(str(self.max_x - self.min_x -1) + "," + str(self.max_y - self.min_y -1) + "\n")

            for x in self.M[self.min_x + 1:self.max_x, self.min_y + 1:self.max_y]:
                for y in x:
                    file.write(str(int(y)) + ",")
                file.write("\n")
            """
            for x in range(self.min_x+1, self.max_x):
                for y in range(self.min_y+1, self.max_y-1):
                    file.write(str( self.M[x][y] ) +",")
                file.write("\n")
            """
    def evaluate(self, x, y, theta):
        r = 1
        A = self.sensor.snapshot(x, y, r)
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                self.set(x + i, y + j, A[i + 1][j + 1])
        return
        self.sensor.evaluate(True)
        for i in range(0, 360, 45):

            d = self.sensor.memory[i]
            _x, _y = next_point(x, y, np.radians(i) - theta, d)
            _x, _y, _ = self.e._get_discrete_position(_x, _y, theta)
            if d < 0.1:
                self.set(int(_x), int(_y), 2)
            else:
                self.set(int(_x), int(_y), 0)
            # print(i, d,(x,y), (_x, _y))

# # todo: aggiustare il salvataggio della matrice. Niente parentesi. prima riga dimensioni della matrice, separate da
# #         sola virgola. ogni riga una riga della matrice, valori separati da sola virgola.
