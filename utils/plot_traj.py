import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.00, 7.00]
plt.rcParams["figure.autolayout"] = True
im = plt.imread("../heightmap.png")
fig, ax = plt.subplots()
im = ax.imshow(im, extent=[0, 1000, 0, 1000])
x = np.array(range(300))
# ax.plot(x, x, ls='dotted', linewidth=2, color='red')
traj = np.loadtxt('../traj_followed.csv',delimiter=',',skiprows=1)
plt.plot(traj[:,2],traj[:,0])
plt.show()