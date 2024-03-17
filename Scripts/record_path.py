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
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf
import math
from fusion_engine_client.parsers import FusionEngineDecoder
from fusion_engine_client.messages import PoseMessage
import threading
import socket
import time
import signal
import sys
from gps import lla_to_utm

TCP_IP = ""
TCP_PORT = 15213  
BUFFER_SIZE = 8
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


msg_decoder = FusionEngineDecoder( # to decode RTK msg
            max_payload_len_bytes=4096, warn_on_unrecognized=False, return_bytes=True)

GPSConnect = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
GPSConnect.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
GPSConnect.connect((TCP_IP, TCP_PORT))
  

#### GPS Callback ##########################
def update_gps():
    global car_state
    while True:
        #print("receiving ...")
        data = GPSConnect.recv(BUFFER_SIZE)
        #print("Stuck?")
        #print(data)
        # data = data.decode("ISO-8859-1")
        if not data:
            print("No data yet")
            time.sleep(0.1)
        try:
            data = msg_decoder.on_data(data)[0][1]
            #print(data)
            if type(data) is PoseMessage:
                global car_state
                print(f"loc: {data.lla_deg}\nvel: {data.velocity_body_mps}")
                car_state['gps_loc'] = data.lla_deg
                car_state['gps_vel'] = data.velocity_body_mps
                #print(car_state)
                # print("recv gps", car_state['gps_loc'])
        except:
            # print(type(data))
            ...

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

def pos_callback(data) :
  global slam_pose_x, slam_pose_y, slam_pose_yaw
  slam_pose_x, slam_pose_y = data.pose.position.x, data.pose.position.y
  _,_,slam_pose_yaw = convert_xyzw_to_rpy(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w)

def pos_callback_amcl(data) :
  global slam_pose_x, slam_pose_y, slam_pose_yaw
  slam_pose_x, slam_pose_y = data.pose.pose.position.x, data.pose.pose.position.y
  _,_,slam_pose_yaw = convert_xyzw_to_rpy(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)

def main(args):
  rospy.init_node('record_path', anonymous=True)
  global tf_listener, pub
  pub = rospy.Publisher('/vesc/joy',Joy)
  # global slam_pose_x, slam_pose_y, slam_pose_yaw
  # slam_pose_x = 0
  # slam_pose_y = 0
  # slam_pose_yaw = 0
  # amcl_pose = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,pos_callback_amcl)
  
  # tf_listener = tf.TransformListener()
  # tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
  path = []
  print("Press enter to record a new point and save it to center_line_recorded.csv, q to quit")
  # TCP To receive RTK info
  gps_thread = threading.Thread(target=update_gps)
  gps_thread.start()
        # logging_thread.join()
  try:
    while(True) :
      print("Waiting")
      time.sleep(0.1)    
  # (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
  #     _,_,yaw = convert_xyzw_to_rpy(rot[0],rot[1],rot[2],rot[3])
      x,y,yaw = car_state['gps_loc'][0], car_state['gps_loc'][1],0.
      print(x,y,yaw)
      path.append([x,y,yaw])
      np.savetxt('center_line_recorded.csv',np.array(path),delimiter=',')
      a = input()
      if a=='q' :
        break
  except KeyboardInterrupt:
    gps_thread.join()
    exit(0)
    # keyboard_thread.join()
    # joystick_thread.join()

if __name__ == '__main__':
    main(sys.argv)
