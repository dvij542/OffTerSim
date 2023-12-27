import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

ep_no = 350
for n in range(1,100) :
    fname = 'eg_comps/eg_'+str(n)+'_'+str(ep_no)+'.txt'
    arr = np.loadtxt(fname)
    arr_gt = arr[0,:].reshape((5,11))
    arr_pred = arr[1,:].reshape((5,11))

    pts_gt = []
    pts_pred = []
    for i in range(5) :
        for j in range(11) :
            t = (-30 + 6*j)*math.pi/180.
            x = (i+1)*math.sin(t)
            y = (i+1)*math.cos(t)
            z1 = arr_gt[i,j]
            z2 = arr_pred[i,j]
            pts_gt.append([x,y,z1])
            pts_pred.append([x,y,z2])

    pts_gt = np.array(pts_gt)
    pts_pred = np.array(pts_pred)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot
    ax.scatter(pts_gt[:,0], pts_gt[:,1], pts_gt[:,2], c='b', marker='o')
    ax.scatter(pts_pred[:,0], pts_pred[:,1], pts_pred[:,2], c='r', marker='o')

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show the plot
    plt.show()