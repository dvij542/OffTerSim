import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

FILE_NAME = 'center_line_recorded.csv'
DS = 0.1
# Define some points:
points = np.loadtxt(FILE_NAME,delimiter=' ')[:,:2] # a (nbre_points x nbre_dim) array

# Linear length along the line:
distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
# print(distance)
total_dist = distance[-1]
distance = np.insert(distance, 0, 0)/distance[-1]

# Interpolation for different methods:
interpolations_methods = ['slinear', 'quadratic', 'cubic']
alpha = np.linspace(0, 1, int(total_dist//DS))

interpolated_points = {}
for method in interpolations_methods:
    interpolator =  interp1d(distance, points, kind=method, axis=0)
    interpolated_points[method] = interpolator(alpha)

# Graph:

plt.figure(figsize=(7,7))
cubic_curve = interpolated_points['slinear']
for method_name, curve in interpolated_points.items():
    print(curve.shape)
    plt.plot(*curve.T, '-', label=method_name)

plt.plot(*points.T, 'ok', label='original points')
plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')
np.savetxt('center_line.csv',cubic_curve,delimiter=',')
plt.show()