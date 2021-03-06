#!/usr/bin/env python2.7
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## input --> list of points (x,y,z)

class odom():
    def __init__(self):
        self.a = np.matrix([[2, -2, 1, 1],
                            [-3, 3, -2, -1],
                            [0, 0, 1, 0],
                            [1, 0, 0, 0]
                            ])

    def init(self, points):
        self.points_x = points[:, 0]
        self.points_y = points[:, 1]
        self.points_z = points[:, 2]
        self.time = np.arange(0, 1, 0.01)

    def getGain(self, p1, p3):
        return 0.5 * (p3 - p1)

    def calcGains(self, points):
        gain = list()
        for i in range(0, len(points)):
            if i == 0:
                gain.append(points[0] * 1.0)
            elif i == len(points) - 1:
                gain.append(points[i] * 1.0)
            else:
                gain.append(self.getGain(points[i - 1], points[i + 1]))
        return gain

    def poly(self, points):
        d = list()
        gains = self.calcGains(points)
        for i in range(0, len(points) - 1):
            C = np.transpose(np.matrix([points[i], points[i + 1], gains[i], gains[i + 1]]))
            for t in self.time:
                d.append(float(np.matrix([t ** 3, t ** 2, t, 1]) * self.a * C))
        return d

    def poly_sample(self, points, time, slot):
        gains = self.calcGains(points)

        C = np.transpose(np.matrix([points[slot], points[slot + 1], gains[slot], gains[slot + 1]]))
        return float(np.matrix([time ** 3, time ** 2, time, 1]) * self.a * C)

    def getValue(self, time):
        time *= len(self.points_x) - 1
        slot = int(time)
        return np.array([self.poly_sample(self.points_x, time - slot, slot),
                         self.poly_sample(self.points_y, time - slot, slot),
                         self.poly_sample(self.points_z, time - slot, slot)])


if __name__ == '__main__':
    points = np.array([[1.0, 2.0, 3.0],
                       [-1.5, 4.5, -8.5],
                       [9.0, 2.0, 1.0],
                       [8.0, 5.0, 2.0],
                       [1.1, 1.3, 2.1]
                       ])
    spline = odom()
    spline.init(points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    data = np.array([0, 0, 0])
    t = np.arange(0, 1, 0.01)
    for i in t:
        data = np.vstack((data, spline.getValue(i)))

    ax.plot(xs=data[1:,0], ys=data[1:,1], zs=data[1:,2])
    #plt.plot(data[1:, 0])
    ax.scatter(points[0:,0],points[0:,1],points[:,2])
    plt.show()
