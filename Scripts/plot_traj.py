import numpy as np
import matplotlib.pyplot as plt
from gps import lla_to_utm

# Load data
data = np.loadtxt('center_line_recorded.csv',delimiter=',')

data_new = np.zeros((data.shape[0],4))
origin = lla_to_utm(data[0,0], data[0,1])
for i in range(len(data)):
    x, y = lla_to_utm(data[i,0], data[i,1])
    data_new[i,0] = -x
    data_new[i,1] = -y
    data_new[i,2] = data[i,0]
    data_new[i,3] = data[i,1]

np.savetxt('center_line_recorded.csv',data_new,delimiter=',')
plt.plot(-(data_new[:,0]-origin[0]), -(data_new[:,1]-origin[1]), 'r')
plt.axis('equal')
plt.show()
