from __future__ import division

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.linalg import solve_banded

data_points = np.array([
  # [1, 0],
  # [3, 1],
  # [5, 0.5],
  # [7, 1],
  # [9, 0]
 [1.0, 2.0, 3.0],
 [-1.5, 4.5, -8.5],
 [9.0, 2.0, 1.0],
 [8.0, 5.0, 2.0],
  [1.1, 3.3, 2.1],
  [1.1, 3.3, 5.1],
  [5, -5, 5.1]
])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax = fig.add_subplot(111)
#ax2 = fig.add_subplot(111)

def drange(start, stop, step):
  r = start
  while r < stop:
    yield r
    r += step

num_cps = data_points.shape[0] - 1;

ab = np.matrix([
  np.tile(1, num_cps),
  np.tile(4, num_cps),
  np.tile(1, num_cps)
])

ab[0,0] = 0
ab[1,0] = 2
ab[1, num_cps - 1] = 7
ab[2, num_cps - 1] = 0

b = np.empty([num_cps, data_points.shape[1]])

for i in range(0, num_cps - 1):
  b[i] = 4 * data_points[i] + 2 * data_points[i + 1]

b[0] = data_points[0] + 2 * data_points[1]
b[num_cps - 1] = 8 * data_points[num_cps - 1] + data_points[num_cps]

print ab
print b

control_points1 = solve_banded((1, 1), ab, b )

print control_points1

control_points2 = np.empty([num_cps, data_points.shape[1]])

for i in range(0, num_cps):
  if i < num_cps - 1:
    control_points2[i] = 2 * data_points[i + 1] - control_points1[i + 1]
  else:
    control_points2[i] = (data_points[num_cps] + control_points1[num_cps - 1])/np.tile(2, data_points.shape[1])

def to_weighted(p, w):
  return np.append(p, np.tile(1, (p.shape[0], 1)), axis=1) * np.tile(w.reshape(p.shape[0], 1), p.shape[1] + 1)

def bezier(cp, w, n):
  result = np.empty([n, cp.shape[1]])

  for k in range(0, n):
    t = k / (n - 1)
    work = to_weighted(cp, w)
    frac_a = np.tile((1 - t), work.shape[1])
    frac_b = np.tile(t, work.shape[1])

    for j in range(cp.shape[0] - 1, -1, -1):
      for i in range(0, j):
        work[i] =  frac_a * work[i] + frac_b * work[i + 1] 

    result[k] = work[0][0:cp.shape[1]] / np.tile(work[0][cp.shape[1]], cp.shape[1])
    # print k, work[0]

  return result

def bezier_better(dp, cp1, cp2, n_each):
  result = np.empty([(n_each) * (dp.shape[0] - 1) + 1, dp.shape[1]])

  for i in range(0, dp.shape[0] - 1):
    points = np.array([dp[i], cp1[i], cp2[i], dp[i+1]])
    b = bezier(points, np.tile(1, 4), n_each + 1)

    result[i * n_each:(i+1)*n_each] = b[:-1]

  result[-1] = dp[-1]

  return result

# print result

#ax.scatter(np.take(data_points, 0, axis=1), np.take(data_points, 1, axis=1));
# ax.scatter(np.take(control_points, 0, axis=1), np.take(control_points, 1, axis=1), np.take(control_points, 2, axis=1));
ax.plot(np.take(data_points, 0, axis=1), np.take(data_points, 1, axis=1), np.take(data_points, 2, axis=1));
# result1 = bezier(control_points, weights, 100)
# ax.plot(np.take(result1, 0, axis=1), np.take(result1, 1, axis=1), np.take(result1, 2, axis=1));
# result2 = bezier(data_points, np.tile(1, data_points.shape[0]), 100)
# ax.plot(np.take(result2, 0, axis=1), np.take(result2, 1, axis=1), np.take(result2, 2, axis=1), color='red');

result3 = bezier_better(data_points, control_points1, control_points2, 20)
print result3
ax.plot(np.take(result3, 0, axis=1), np.take(result3, 1, axis=1), np.take(result3, 2, axis=1), color='black');


#ax2.plot(range(0, n), np.take(frac_test, 0, axis=1));
#ax2.plot(range(0, n), np.take(frac_test, 1, axis=1));
#ax2.plot(range(0, n), np.take(frac_test, 2, axis=1));

#ax.scatter(np.take(pa_test, 0, axis=1), np.take(pa_test, 1, axis=1), np.take(pa_test, 2, axis=1));
#ax.scatter(np.take(pb_test, 0, axis=1), np.take(pb_test, 1, axis=1));

#print pa_test
#print pb_test
#print result


plt.show()
