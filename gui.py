#
#
#

import sys
import math

# import PyQt5
from PyQt5 import QtGui, QtCore, QtWidgets
from mobile_robot import *
from polar_control import *
import numpy as np
from environment import *


class MainWindow(QtWidgets.QWidget):

    def __init__(self):
        super(MainWindow, self).__init__()

        self.initUI()

    def initUI(self):
        self.e = Environment(5, 5)

        '''
        Number of physical point per cell 100
        Number of cell 5
        Number of virtual point per cell d * 1000
        '''

        self.scale = 50
        self.setGeometry(0, 0, 700, 600)
        self.setWindowTitle('Robot 2D Simulator')
        self.show()
        self.robot_pic = QtGui.QPixmap("mobile_robot_2d.png")
        self.obstacle_pic = QtGui.QPixmap("obstacle-2d.png")
        self.over_pic = QtGui.QPixmap("over_2d.png")
        self.delta_t = 1e-4  # 0.1ms of time-tick
        self.t = 0

        self._timer_painter = QtCore.QTimer(self)
        self._timer_painter.start(self.delta_t * 10000)
        self._timer_painter.timeout.connect(self.go)

        self.robot = Robot(
            self.e,  # env
            0.1      # robot diameter
        )
        self.e.set_robot(self.robot)
        self.e.get_obstcle_position()
        self.position_array = []
        self.snap_counter_reset()

    def snap_counter_reset(self):
        self.snap_counter = 400

    def go(self):
        self.robot.evaluate(self.delta_t)
        self.t += self.delta_t
        self.update()  # repaint window

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setPen(QtGui.QColor(255, 255, 255))
        qp.setBrush(QtGui.QColor(255, 255, 255))
        qp.drawRect(event.rect())



        (x, y, theta) = self.robot.get_pose()


        qp.setPen(QtCore.Qt.black)

        #bottom
        qp.drawLine(50, self.e.n*self.scale+50, self.e.m*self.scale+50, self.e.n*self.scale+50)
        #left(?)
        qp.drawLine(50, self.e.n*self.scale+50, 50, 50)
        #top
        qp.drawLine(50, 50, self.e.m*self.scale+50, 50)
        #right
        qp.drawLine(self.e.m*self.scale+50, 50, self.e.m*self.scale+50, self.e.n*self.scale+50)

        qp.drawText(self.e.m*self.scale+100, 60, "X  = %6.3f m" % (x))
        qp.drawText(self.e.m*self.scale+100, 80, "Y  = %6.3f m" % (y))
        qp.drawText(self.e.m*self.scale+100, 100, "Th = %6.3f deg" % (math.degrees(theta)))
        qp.drawText(self.e.m*self.scale+100, 120, "T  = %6.3f s" % (self.t))




        s = self.robot_pic.size()


        _x = x * self.scale*10
        _y = y * self.scale*10
        __x = _x * 0.1 / self.robot.d
        __y = _y * 0.1 / self.robot.d
        x_pos = 50 + __x - self.scale / 2
        y_pos =  50 + (self.e.n - y / self.robot.d)*self.scale -self.scale/2  #+self.scale/2  + (self.e.n) * self.scale - y*self.scale/self.robot.d# - (__y - self.scale / 2)


        for p in self.position_array:
            qp.drawPixmap(p[0], p[1], self.scale, self.scale, self.over_pic)

        # Troppo pesante aggiungere una posizione per ogni iterazione
        # ne aggiungiamo solo una ogni volta che scatta il timer
        if self.snap_counter == 0:
            self.position_array.append((x_pos, y_pos))
            self.snap_counter_reset()
        else:
            self.snap_counter -= 1

        obstacles = self.e.get_obstcle_position()
        for obstacle in obstacles:
            #print(obstacle)
            #qp.drawPixmap( obstacle[0] * self.scale+ 50,  (self.e.n - obstacle[1])*100 -50, self.obstacle_pic)
            qp.drawRect(obstacle[0] * self.scale + 50,  50 + (self.e.n -1 - obstacle[1])*self.scale, self.scale, self.scale)
            #qp.drawPixmap(450, 450, self.obstacle_pic)

        #qp.drawRect(x_pos , y_pos, self.scale, self.scale)

        t = QtGui.QTransform()
        t.translate(x_pos + self.scale / 2, y_pos + self.scale / 2)
        t.rotate(-math.degrees(theta))
        t.translate(-(x_pos + self.scale / 2), - (y_pos + self.scale / 2))

        qp.setTransform(t)
        #qp.drawPixmap(x_pos, y_pos, self.robot_pic)
        qp.drawPixmap(x_pos, y_pos,  self.scale, self.scale, self.robot_pic)
        qp.setTransform(QtGui.QTransform())






        qp.end()


def main():
    app = QtWidgets.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
