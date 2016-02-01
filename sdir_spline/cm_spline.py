#!/usr/bin/env python3
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


time_ = np.arange(0, 1, 0.01)

points_x = np.array([1.0,-1.5,9.0,8.0,2,4,7])


def getGain(p1,p3):
    return 0.5*(p3-p1)


a = np.matrix([[2,-2,1,1],
               [-3,3,-2,-1],
               [0,0,1,0],
               [1, 0, 0, 0]
               ])



#for i in time_:
#    d.append(float(time(i)*a*np.transpose(points_x)))



def calcGains(points):
    gain=list()
    for i in range(0,len(points)):
        if i==0:
            gain.append(points[0]*1.0)
        elif i==len(points)-1:
            gain.append(points[i]*1.0)
        else:
            gain.append(getGain(points[i-1],points[i+1]))
    return gain


def calcValue(points):
    d=list()
    gains = calcGains(points)
    for i in range(0,len(points)-1):
        C = np.transpose(np.matrix([points[i],points[i+1],gains[i],gains[i+1]]))
        for t in time_:
            d.append(float(np.matrix([t**3,t**2,t,1])*a*C))
    return d






data = calcValue(points_x)



plt.plot(np.arange(0,len(points_x)-1, 0.01),data)
plt.plot(np.arange(0, len(points_x), 1),points_x,'ro')
plt.ylabel('some numbers')
plt.show()