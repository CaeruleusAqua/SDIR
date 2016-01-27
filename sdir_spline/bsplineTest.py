import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


time_ = np.array([1.0, 2.0, 3.0, 4.0, 5.0])

points_x = np.array([1.0,-1.5,9.0,8.0,1.1])
points_y = np.array([2.0, 4.5,2.0,5.0,1.3])
points_z = np.array([3.0,-8.5,1.0,2.0,2.1])

n = 5
m = 3
_t = []




def calculateB(i,k,t):
    if k==0:
        if _t[i]<= t < _t[i+1]:
            return 1
        else:
            return 0
    else:
        a = (_t[i+k]-_t[i])
        b = (_t[i+k+1]-_t[i+1])
        if a == 0 & b == 0:
            return 0
        elif a == 0:
            return ((_t[i+k+1]-t)/b) * calculateB(i+k,k-1,t)
        elif b == 0:
            return ((t-_t[i])/a) * calculateB(i,k-1,t)
        else:
            return ((t-_t[i])/a) * calculateB(i,k-1,t) + ((_t[i+k+1]-t)/b) * calculateB(i+k,k-1,t)
 
def calculateT(j):
    if j<=m:
        return 0
    elif m+1 <= j <= n:
        return j-n
    elif j > n:
        return n-m+1
    
def calculateTs():
    for i in range(0, n+m+1):
        _t.append(calculateT(i))
    
def calculateKoordinate(_k, t):
    k = 0
    for i in range(0,n):
       k = k + _k[i]*calculateB(i,m,t)
    return k
           
           
calculateTs()

bx = 0

for i in range(0,4):
   bx = bx + points_x[i] * calculateB(i,m,1) 

#for point in points_x:
#    for t in _t:
#        for i in range (0,n):
#            Bx = Bx + point*calculateB(i,m,t)