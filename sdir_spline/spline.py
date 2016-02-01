#!/usr/bin/env python3
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


time_ = np.array([0.0, 1.0, 2.0, 3.0, 4.0])

points_x = np.array([1.0,-1.5,9.0,8.0,1.1])
points_y = np.array([2.0, 4.5,2.0,5.0,1.3])
points_z = np.array([3.0,-8.5,1.0,2.0,2.1])



def getCoeff(points, time):

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

    return points,b,C,d


def poly(x,a,b,c,d,i):
    return np.array(a[i] + b[i] * (x - time_[i]) + c[i] * (x - time_[i]) ** 2 + d[i] * (x - time_[i]) ** 3)[0]

a,b,c,d = getCoeff(points_x,time_)
print("Koeef: " + str(a) + " , " + str(b) + " , " + str(c))


data_x=np.array([])
for i in range(0,len(a)-1):
    t = np.arange(time_[i], time_[i+1], 0.01)
    data_x=np.hstack((data_x, poly(t, a, b, c, d, i)))



a,b,c,d = getCoeff(points_y,time_)

data_y=np.array([])
for i in range(0,len(a)-1):
    t = np.arange(time_[i], time_[i+1], 0.01)
    data_y=np.hstack((data_y, poly(t, a, b, c, d, i)))



a,b,c,d = getCoeff(points_z,time_)

data_z=np.array([])
for i in range(0,len(a)-1):
    t = np.arange(time_[i], time_[i+1], 0.01)
    data_z=np.hstack((data_z, poly(t, a, b, c, d, i)))



#line, = plt.plot(t,data)
ax.plot(xs=data_x, ys=data_y, zs=data_z)
ax.scatter(points_x,points_y,points_z)
plt.show()
#plt.plot(time_, points_, 'ro')
#plt.show()
