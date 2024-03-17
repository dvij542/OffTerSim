#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('racecar')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import geometry_msgs.msg
import tf
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import torch
import torch.nn as nn
# import transforms
from torch.utils.data import DataLoader, Dataset
# import resnet as models
import os 
from scipy.stats import norm
import time
from scipy.interpolate import griddata
import ros_numpy
from PIL import Image
from fusion_engine_client.parsers import FusionEngineDecoder
from fusion_engine_client.messages import PoseMessage
import threading
import socket
from gps import lla_to_utm

########## INSTRUCTIONS ##########
# 1. Hole filling for missing values from depth image
# Observation inputs:-
#   0-15. Depth observations from LIDAR (angles -15 to +15 degrees, resolution 2 degrees): 182 
#           Format:- (0,2,-2,4,-4,...,90,-90) degrees and each input contains (mask,range/5.)
#   16. State: 12 (speed,vx,vy,vz(0.),roll(in degrees)/90.,omega(yaw rate),pitch(in degrees)/90.,
#                  last throttle,last brake,heading angle error,lateral displacement,n_collisions(0))

# Change these params
RUN_NO = 7
RUN_ID = 0
USE_GPS = False
STRAIGHT_LINE = True
SAFEGUARD = True
LANE_WIDTH = 1.
lambda_ = 2.
K = 6.
freq = 2.
inflation_dist = 0.4#5.*math.pi/180.
thres = 2.
steer_factor = 1.2
alpha = 30.
steer_cost = -0.8
ttl = np.loadtxt('traj_rr.csv',delimiter=' ')
SET_ORIGIN = False
INIT_YAW = 0.#math.pi/2.
L = 0.3 # Length of the wheelbase of the vehicle used for kinematic vehicle model
EXPERT_TRACKING = 'pure_pursuit' # Select from 'pure_pursuit', 'mpc'
if EXPERT_TRACKING == 'pure_pursuit' :
  LOOKAHEAD_DIST = 0.8 # Lookahead distance for Pure pursuit
forward_speed = 0.15 # Target speed
WAYPOINT_GAP = 0.1 # Set equidistant waypoints on the interpolated centre line at this much gap between consecutive points
car_state = dict(
                    date=None,
                    time=None, 
                 gps_loc=(0,0,0),
                 gps_vel=None,
                 left_rgb=None,
                 right_rgb=None,
                 lidar=None,
                 imu=None,
                 zed2_odom=None,
                 vesc_odom=None,
                 throttle=0.0,
                 steering=0.0,
            )



def has_crossed_end_line(x,y) :
  # print("Received ", x,y)
  return (y>10.7)

def pure_pursuit(ttl,state) :
  x,y,yaw = state[0],state[1],state[2]
  dists = (ttl - np.array([[x,y]]))
  #print(dists)
  dists = dists[:,0]**2 + dists[:,1]**2
  mini = np.argmin(dists)
  targeti = min((mini + int(LOOKAHEAD_DIST/WAYPOINT_GAP))%dists.shape[0],len(ttl)-1)
  tarx,tary = ttl[targeti,0],ttl[targeti,1]
  diffx,diffy = tarx-x,tary-y
  dx,dy = diffx*math.cos(yaw) + diffy*math.sin(yaw),diffy*math.cos(yaw)-diffx*math.sin(yaw)
  print("pure pursuit :", mini,diffx,diffy,dx,dy)
  str_val = 2*L*dy/(dx**2+dy**2)
  return str_val
  
def find_min_dist(center_line,p) :
    x,y = p[0], p[1]
    dists = (center_line[:,:2]-np.array([[x,y]]))
    dist = dists[:,0]**2 + dists[:,1]**2
    # print(min(dist))
    mini = np.argmin(dist)
    vals = []
    if mini>0 :
        x1,y1 = center_line[mini-1,0],center_line[mini-1,1]
        x2,y2 = center_line[mini,0],center_line[mini,1]
        a,b,c = -(y2-y1), (x2-x1),y2*x1-y1*x2 
        ym = (a*(a*y-b*x)-b*c)/(a**2+b**2)
        xm = (-b*(a*y-b*x)-a*c)/(a**2+b**2)
        if (xm>min(x1,x2) and xm<max(x1,x2)) or (ym>min(y1,y2) and ym<max(y1,y2)) :
          vals.append(abs((a*x+b*y+c)/(math.sqrt(a**2+b**2))))
        else :
          vals.append(min(math.sqrt((x-x1)**2+(y-y1)**2),math.sqrt((x-x2)**2+(y-y2)**2)))
    if mini < len(center_line)-1 :
        x1,y1 = center_line[mini,0],center_line[mini,1]
        x2,y2 = center_line[mini+1,0],center_line[mini+1,1]
        a,b,c = -(y2-y1), (x2-x1),y2*x1-y1*x2 
        ym = (a*(a*y-b*x)-b*c)/((a**2+b**2))
        xm = (-b*(a*y-b*x)-a*c)/(a**2+b**2)
        if (xm>min(x1,x2) and xm<max(x1,x2)) or (ym>min(y1,y2) and ym<max(y1,y2)) :
          vals.append(abs((a*x+b*y+c)/(math.sqrt(a**2+b**2))))
        else :
          vals.append(min(math.sqrt((x-x1)**2+(y-y1)**2),math.sqrt((x-x2)**2+(y-y2)**2)))
    # print("aa : ", min(vals))
    return min(vals)

def get_frenet_state(center_line,state) :
  x,y,yaw = state[0],state[1],state[2]
  dists = (center_line - np.array([[x,y]]))
  dists = dists[:,0]**2 + dists[:,1]**2
  mini = np.argmin(dists)  
  if mini < len(center_line)-1 : 
    closest_angle_vector = center_line[mini+1,:]-center_line[mini,:]
  else :
    closest_angle_vector = center_line[mini,:]-center_line[mini-1,:]
  vec1 = np.array([x,y])-center_line[mini,:2]
  vec2 = np.array(closest_angle_vector)
  val = vec2[0]*vec1[1] - vec2[1]*vec1[0]
  # x_ = math.sqrt(dists[mini])*val/abs(val)
  x_ = find_min_dist([x,y])*val/abs(val)
  print("Lat err : ",x_)
  if mini < len(center_line)-2 : 
    theta = yaw - math.atan2(center_line[mini+1,1]-center_line[mini,1],center_line[mini+1,0]-center_line[mini,0])
  else :
    theta = yaw - math.atan2(center_line[mini,1]-center_line[mini-1,1],center_line[mini,0]-center_line[mini-1,0])

  while theta > math.pi :
    theta -= 2*math.pi
  while theta < -math.pi :
    theta += 2*math.pi
      
def pos_callback(data) :
  global slam_pose_x, slam_pose_y, slam_pose_yaw
  slam_pose_x, slam_pose_y = data.pose.position.x, data.pose.position.y
  _,_,slam_pose_yaw = convert_xyzw_to_rpy(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w)

def pos_callback_amcl(data) :
  global slam_pose_x, slam_pose_y, slam_pose_yaw
  slam_pose_x, slam_pose_y = data.pose.pose.position.x, data.pose.pose.position.y
  _,_,slam_pose_yaw = convert_xyzw_to_rpy(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)

def expert_policy(heights) :
    ranges = [[0,4],[4,7],[7,11]]
    cum_heights = []
    for i in range(3) :
        curr_cum_height = 0
        for j in range(ranges[i][0],ranges[i][1]) :
          for k in range(heights.shape[0]) :
            curr_cum_height += heights[k,j]/(ranges[i][1]-ranges[i][0])
        cum_heights.append(-curr_cum_height)
    # print(cum_heights)
    return 2 - np.argmax(cum_heights)

def convert_to_pt_cloud(img,mask=None,ang_wide=75.,dist_factor=3., roll=0.) :
    pts = []
    for i in range(0,img.shape[0],3) :
        for j in range(0,img.shape[1],3) :
            angle_vert = -(i - img.shape[0]/2.)*(ang_wide/img.shape[0])
            angle_hor = (j - img.shape[1]/2.)*(ang_wide/img.shape[0])
            y = img[i,j]*dist_factor/255.
            x = y*math.tan(angle_hor*math.pi/180.)
            z = y*math.tan(angle_vert*math.pi/180.)
            if mask is not None and mask[i,j]==1 :
              pts.append([x,y,z])
    return pts

def interpolate_point(a, b, data):
    # Extract x, y, and f(x, y) values from the data
    x = data[:, 0]
    y = data[:, 1]
    values = data[:, 2]

    # Perform 2D linear interpolation
    interpolated_value = griddata((x, y), values, (a, b), method='nearest')
    # print(interpolated_value)
    return interpolated_value

def get_grid_pts(pcl,angle_max=37.5*math.pi/180.,lds=np.array([.4,.5,.6]),npts=11) :
    # print(pcl)
    pcl = np.array(pcl)
    print(pcl.shape)
    # if np.sum(pcl) < 0.001 :
    #     return np.array([0.]*11)
    pts_all = []
    for ld in lds :
        pts_ = []
        for a in np.arange(-angle_max,angle_max-0.001,2*angle_max/npts) :
            pt = (ld*math.sin(a),ld*math.cos(a))
            # print(arr.shape)
            try:
                z = interpolate_point(pt[0],pt[1],np.array([pcl[:,0],pcl[:,1],pcl[:,2]]).T)
            except:
                print("Not feasible solution")
                return np.array([0.]*11)
            # print(a,z)
            x = pt[0]
            y = pt[1]
            pts_.append(z)
        pts_all.append(pts_)
    pts_all = np.array(pts_all)
    # print(pts_int)
    return pts_all

def convert_xyzw_to_rpy(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

def get_optimal_control(steer_ref,v,theta,x,curvature,max_steering=0.34) :
    # theta = -theta
    # print(steer_ref,v,theta,x,curvature)
    if SAFEGUARD==False :
        return steer_ref
    # return steer_ref
    min_steer = -max_steering
    steer_var = 1
    steer = np.arange(-max_steering,max_steering,0.01)
    ax = 0
    h_left = LANE_WIDTH/2. - x
    hd_left = -v*math.sin(theta)
    hdd_left = -ax*math.sin(theta)-v**2*math.cos(theta)*steer/L + v**2*curvature
    h_right = LANE_WIDTH/2. + x
    hd_right = v*math.sin(theta)
    hdd_right = ax*math.sin(theta)+v**2*math.cos(theta)*steer/L - v**2*curvature
    costs = (steer-steer_ref)**2/steer_var**2
    costs += ((hdd_left+2*lambda_*hd_left+lambda_**2*h_left) < 0)*alpha*(hdd_left+2*lambda_*hd_left+lambda_**2*h_left)**2
    costs += ((hdd_right+2*lambda_*hd_right+lambda_**2*h_right) < 0)*alpha*(hdd_right+2*lambda_*hd_right+lambda_**2*h_right)**2
    min_steer = steer[np.argmin(costs)] 

    return min_steer

class controller:
  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/velodyne_points",PointCloud2,self.dummy_callback,queue_size=1)
    self.gps_sub = rospy.Subscriber("/gps",PoseStamped,self.gps_callback,queue_size=1)
    self.latest_msg = None
    self.set_origin = SET_ORIGIN
    self.origin = [0.,0.]
    self.x = 0.
    self.y = 0.
    self.yaw = 0.
    rospy.Timer(rospy.Duration(0.1), self.callback)
    self.prevx = 0
    self.latest_gps = None
    self.prevy = 0
    self.prev_cmd = 0
    self.prevyaw = 0
    self.prevt = 0
    self.lat_error = 0.
    self.traj = []
    self.i = 0
    self.steering = 0.

  def dummy_callback(self,data) :
    self.latest_msg = data

  def gps_callback(self,data) :
    print("Received GPS ", rospy.Time.now())
    self.latest_gps = data
    if USE_GPS:
      self.x = data.pose.position.x - self.origin[0]
      self.y = data.pose.position.y - self.origin[1]
      self.yaw = data.pose.orientation.z
      if self.set_origin :
        self.set_origin = False
        self.origin = [self.x,self.y]

  def get_vel(self,cmd) :
    return K*cmd

  def callback(self,_):
    # Get inputs
    data = self.latest_msg
    if data is None :
      return
    print("Started")
    if True :
      now = rospy.Time.now()
      print("Current time of transform : ",float(str(now))/1e9)
      t = float(str(now))/1e9
      (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
      if not USE_GPS:
        _,_,self.yaw = convert_xyzw_to_rpy(rot[0],rot[1],rot[2],rot[3]) 
        self.yaw += INIT_YAW
        self.x,self.y = trans[0],trans[1]
      
      
    # print()
    global cmd
    self.lat_error += self.get_vel(forward_speed)*math.sin(self.yaw+self.steering/2.)*0.1
    # print(time.time())
    pcl_raw = ros_numpy.point_cloud2.pointcloud2_to_array(data)[::10]
    # print(np.array(pcl_raw))
    pts = []
    for pt in pcl_raw :
      pts.append([pt[0],pt[1],pt[2]])
    pcl_raw = np.array(pts)
    
    min_dist = 3
    depth_image = np.ones((16,361))*min_dist
    
    xs = (np.arctan2(pcl_raw[:,1],pcl_raw[:,0])*180./math.pi + 180.).astype(int)
    ys = 31 - (np.arctan2(pcl_raw[:,2],np.linalg.norm(pcl_raw[:,:2],axis=1))*180./math.pi + 16.).astype(int)
    dists = (np.linalg.norm(pcl_raw,axis=1)/0.03).astype(int)
    dists = np.clip(dists,0,255)
    # print(np.min(ys),np.max(ys),self.i)
    depth_image[ys//2,xs] = dists
    self.i += 1
    # if self.i > 10 :
    #   self.i = 0
    #   depth_image = depth_image[:,90:270].astype(int)
    #   print(depth_image.shape)
    #   print(np.min(depth_image),np.max(depth_image))
    #   im = Image.fromarray(depth_image.astype(np.uint8))
    #   im.save('depth_im.jpeg')
    msg_stats = PoseStamped()
    msg_stats.header.stamp = now
    msg_stats.pose.position.x = self.x
    msg_stats.pose.position.y = self.y
    msg_stats.pose.orientation.z = self.yaw
    msg_stats.pose.orientation.x = self.steering
    msg_stats.pose.position.z = self.lat_error
    pub_stats.publish(msg_stats)
    cmd=Joy()
    cmd.header.stamp = now
    cmd.buttons = [0,0,0,0,0,0,1,0,0,0,0]
    max_steering = 0.34

    # grid_pts = get_grid_pts(pcl)
    # steer_code = expert_policy(grid_pts)
    # steering = -(steer_code - 1)*max_steering
        
    # steering = pure_pursuit(ttl[:,:2],(self.x,self.y,self.yaw))
    self.steering = 0.2*math.cos(freq*t)
    self.steering = get_optimal_control(self.steering,forward_speed*6.,self.yaw,self.y,0.)
    print(self.steering, "Done")
    print("Curr pose: ", self.x, self.y, self.yaw)
    # print(grid_pts)
    
    cmd.axes = [forward_speed,forward_speed,(self.steering-0.03)/max_steering,(self.steering-0.03)/max_steering,0.,0.]
    pub.publish(cmd)
    return
    
    
def main(args):
  rospy.init_node('controller', anonymous=True, disable_signals=True)
  global tf_listener, pub, pub_stats
  pub = rospy.Publisher('/vesc/joy',Joy)
  pub_stats = rospy.Publisher('/stats',PoseStamped)
  tf_listener = tf.TransformListener()
  

  # slam_pose = rospy.Subscriber('/slam_out_pose',PoseStamped,pos_callback)
  tf_listener.waitForTransform("/odom", "/base_link", rospy.Time(), rospy.Duration(4.0))
  global start_time
  start_time = time.time()
  ic = controller()
  global cmd
  
  cmd = Joy()
  rospy.spin()
  
  # try:
  #   while True :
  #     # print('Hereee')
  #     # cmd.axes = [0.,forward_speed,(steering-0.1)/max_steering,(steering-0.1)/max_steering,0.,0.]
  #     # print(cmd.axes)
  #     #pub.publish(cmd)
  #     time.sleep(0.1)
  # except KeyboardInterrupt:
  #   print("Shutting down")
  
if __name__ == '__main__':
    main(sys.argv)
