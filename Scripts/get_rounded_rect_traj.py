import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.cm as cm


angle = math.pi/2. # Angle of rotation (in rad)
GRAVITY = 9.8
R = 1. # Radius of circle (in m)
a_v = 0.5 # Straight trajectory (vertical) 
a_h = 4.5 # Straight trajectory (horizontal)
gap = 0.1
dv = 0.8
origin = (-a_h/2.,-a_v/2.-R)
Length = 2*math.pi*R + 2*(a_v+a_h)
traj = []
v_base = math.sqrt(GRAVITY)
for g in np.arange(0.,Length,gap) :
    print(g)
    if g < a_h :
        p = np.array([-a_h/2.,-a_v/2.-R]) + np.array([g,0.])
        theta = 0.
        if g<a_h/2. :
            traj.append([p[0],p[1],theta,g,0.,v_base])#+dv*(g)/(a_h/2.)])
        else :
            traj.append([p[0],p[1],theta,g,0.,v_base])#+dv*(a_h-g)/(a_h/2.)])
    elif g < (a_h + math.pi*R/2.) :
        theta = -math.pi/2. + (g-a_h)/R
        p = np.array([a_h/2.,-a_v/2.]) + np.array([R*math.cos(theta),R*math.sin(theta)])
        print(p)
        traj.append([p[0],p[1],theta+math.pi/2.,g,1./R,v_base])
    elif g < (a_h + math.pi*R/2.+a_v) :
        diff = g - (a_h + math.pi*R/2.)
        p = np.array([a_h/2.+R,-a_v/2.]) + np.array([0.,g - (a_h + math.pi*R/2.)])
        if diff<a_v/2. :
            traj.append([p[0],p[1],math.pi/2.,g,0.,v_base+dv*(diff)/(a_v/2.)])
        else :
            traj.append([p[0],p[1],math.pi/2.,g,0.,v_base+dv*(a_v-diff)/(a_v/2.)])
    elif g < (a_h + math.pi*R/2.+a_v+math.pi*R/2.) :
        diff = g - (a_h + math.pi*R/2.+a_v)
        theta = diff/R
        p = np.array([a_h/2.,a_v/2.]) + np.array([R*math.cos(theta),R*math.sin(theta)])
        traj.append([p[0],p[1],theta+math.pi/2.,g,1./R,v_base])
    elif g < (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h) :
        diff = g - (a_h + math.pi*R/2.+a_v+math.pi*R/2.)
        p = np.array([a_h/2.,a_v/2.+R]) + np.array([-diff,0.])
        traj.append([p[0],p[1],math.pi,g,0.,v_base])
    elif g < (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h+math.pi*R/2.) :
        diff = g - (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h)
        theta = math.pi/2.+diff/R
        p = np.array([-a_h/2.,a_v/2.]) + np.array([R*math.cos(theta),R*math.sin(theta)])
        traj.append([p[0],p[1],theta+math.pi/2.,g,1./R,v_base])
    elif g < (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h+math.pi*R/2.+a_v) :
        diff = g - (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h+math.pi*R/2.)
        p = np.array([-a_h/2.-R,a_v/2.]) + np.array([0.,-diff])
        if diff<a_v/2. :
            traj.append([p[0],p[1],3.*math.pi/2.,g,0.,v_base+dv*(diff)/(a_v/2.)])
        else :
            traj.append([p[0],p[1],3.*math.pi/2.,g,0.,v_base+dv*(a_v-diff)/(a_v/2.)])
    elif g < (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h+math.pi*R/2.+a_v+math.pi*R/2.) :
        diff = g - (a_h + math.pi*R/2.+a_v+math.pi*R/2.+a_h+math.pi*R/2.+a_v)
        theta = math.pi+diff/R
        p = np.array([-a_h/2.,-a_v/2.]) + np.array([R*math.cos(theta),R*math.sin(theta)])
        traj.append([p[0],p[1],theta+math.pi/2.,g,1./R,v_base])

traj.append([traj[0][0],traj[0][1],traj[0][2],traj[0][3],traj[0][4],traj[0][5]])

ref_traj = np.array(traj) - np.array([[origin[0],origin[1],0.,0.,0.,0.]])
rot_mat = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
ref_traj[:,:2] = np.dot(rot_mat,ref_traj[:,:2].T).T
plt.plot(ref_traj[:,0],ref_traj[:,1])
K1 = K2 = -19
mu_orig = 1.1
factor = math.sqrt(-(K1+K2)*0.39/(mu_orig*9.8))
print(factor)
ref_speeds = factor*ref_traj[:,-1]
cmap = plt.get_cmap('jet')
norm = plt.Normalize(min(ref_speeds), max(ref_speeds))
for j in range(1, len(ref_traj)):
    speed = ref_speeds[j-1]
    x0, y0 = ref_traj[j-1,0], ref_traj[j-1,1]
    x1, y1 = ref_traj[j,0], ref_traj[j,1]
    plt.plot([x0, x1], [y0, y1], '-', color=cmap(norm(speed)))
plt.axis('equal')
mappable = cm.ScalarMappable(norm, cmap)
mappable.set_array(ref_speeds)
# create the colorbar
cb = plt.colorbar(mappable)    
# traj_out = np.loadtxt('traj_rr_out.csv') - np.array([[origin[0],origin[1],0.,0.,0.,0.]])
# traj_in = np.loadtxt('traj_rr_in.csv') - np.array([[origin[0],origin[1],0.,0.,0.,0.]])
# plt.plot(traj_out[:,0],traj_out[:,1],color='r')
# plt.plot(traj_in[:,0],traj_in[:,1],color='r')
cb.set_label('Speeds (in m/s)')
# add some additional information
plt.title("Reference racing line")
plt.xlabel('x (in m)')
plt.ylabel('y (in m)')
# plt.savefig('videos/'+RUN_NAME+'/racing_lines/'+str(i//5)+'.png')
plt.savefig('rounded_rect_ref.png')
plt.show()
plt.close()
np.savetxt('traj_rr.csv',ref_traj)

