#!/usr/bin/env python2.7
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## input --> list of points (x,y,z)

class odom():
    def __init__(self, points):
        self.init(points)

    def __init__(self):
        pass

    def plot(self, data):
        pass

    def getCoeff(self, points, time):

        dim = len(points)

        delta = np.zeros(dim - 1)

        for i in range(dim - 1):
            delta[i] = points[i + 1] - points[i]

        eps = np.zeros(dim - 1)

        for i in range(dim - 1):
            eps[i] = time[i + 1] - time[i]

        Y = np.zeros(dim)
        for i in range(1, dim - 1):
            Y[i] = 3 * ((delta[i] / eps[i]) - (delta[i - 1] / eps[i - 1]))

        Y = np.matrix(Y).transpose()

        M = np.zeros([dim, dim])

        M[0, 0] = 1
        M[dim - 1, dim - 1] = 1

        for i in range(1, dim - 1):
            M[i, i - 1] = eps[i - 1]
            M[i, i] = 2 * (eps[i - 1] + eps[i])
            M[i, i + 1] = eps[i]

        C = inv(M) * Y

        b = np.empty(dim - 1)
        d = np.empty(dim - 1)

        for i in range(0, dim - 1):
            b[i] = (delta[i] / eps[i]) - (1 / 3.) * (2.0 * C[i] + C[i + 1]) * eps[i]
            d[i] = (C[i + 1] - C[i]) / (3.0 * eps[i])

        return points, b, C, d

    def init(self, points):
        self.points_x = points[:, 0]
        self.points_y = points[:, 1]
        self.points_z = points[:, 2]
        self.time = np.arange(0, len(self.points_x), 1, np.dtype(np.float64))

        self.coeff_x = self.getCoeff(self.points_x, self.time)
        self.coeff_y = self.getCoeff(self.points_y, self.time)
        self.coeff_z = self.getCoeff(self.points_z, self.time)

    def poly(self, x, a, b, c, d, i):
        return np.array(a[i] + b[i] * (x - self.time[i]) + c[i] * (x - self.time[i]) ** 2 + d[i] * (x - self.time[i]) ** 3)[0]

    def poly_sample(self, x, a, b, c, d, i):
        return np.array(a[i] + b[i] * (x - self.time[i]) + c[i] * (x - self.time[i]) ** 2 + d[i] * (x - self.time[i]) ** 3)[0][0]

    def getValue(self, time):  # time between 0 and 1
        time *= self.time[-1]
        slot = 0
        for i in range(len(self.time)):
            if self.time[i] >= time:
                slot = i - 1
                break
        slot=max(slot,0)

        print slot
        return np.array([self.poly_sample(time, self.coeff_x[0], self.coeff_x[1], self.coeff_x[2], self.coeff_x[3], slot),
                         self.poly_sample(time, self.coeff_y[0], self.coeff_y[1], self.coeff_y[2], self.coeff_y[3], slot),
                         self.poly_sample(time, self.coeff_z[0], self.coeff_z[1], self.coeff_z[2], self.coeff_z[3], slot)])


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

    data=np.array([0,0,0])
    t = np.arange(0, 1.01, 0.01)
    for i in t:
        data=np.vstack((data, spline.getValue(i)))


    ax.plot(xs=data[1:,0], ys=data[1:,1], zs=data[1:,2])
    ax.scatter(points[:,0],points[:,1],points[:,2])
    plt.show()
