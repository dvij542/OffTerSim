import numpy as np
import torch
from mlagents_envs.environment import UnityEnvironment,ActionTuple
from mlagents_envs.base_env import ActionSpec, TerminalSteps
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import random
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import math
from PIL import Image
from sensor_msgs.msg import Image as Image_ros
from scipy.interpolate import griddata

import matplotlib.pyplot as plt
from threading import Thread
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, TransformStamped
import tf2_ros
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField

steer_cmd = 0
throttle_cmd = 0

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.subscription = self.create_subscription(
            AckermannDrive,
            'cmd',
            self.cmd_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_pose = self.create_publisher(
            PoseStamped,
            'car_pose',
            10)
        self.publisher_twist = self.create_publisher(
            TwistStamped,
            'car_twist',
            10)
        self.publisher_accel = self.create_publisher(
            AccelStamped,
            'car_accel',
            10)
        self.publisher_img = self.create_publisher(Image_ros, 'front_camera', 10)
        self.env = UnityEnvironment(file_name="ros2-env-v2/sim", seed=1, side_channels=[],worker_id=0,log_folder='logs/')#,no_graphics=True)
        # print("Started?")
        self.env.reset()
        self.fps = 10.
        # print("Started?")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.behavior_name = list(self.env.behavior_specs)[0]
        
        # print("Started?")
        self.subscription  # prevent unused variable warning    
        self.timer = self.create_timer(0.1, self.publish_data)
        self.publisher_scan = self.create_publisher(
            LaserScan,
            'scan',
            10)
        self.base_idx = 1
        self.publisher = self.create_publisher(PointCloud2, 'pcl', 10)
        self.curr_time = time.time()
        self.fps_target = 10.
    
    def publish_scan(self, scan_data):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = -math.pi / 2  # Adjust according to your LiDAR specs
        scan_msg.angle_max = math.pi / 2   # Adjust according to your LiDAR specs
        scan_msg.angle_increment = math.pi / 90  # Adjust according to your LiDAR specs
        scan_msg.time_increment = 0.#1.0 / 1000  # Adjust according to your LiDAR specs
        scan_msg.range_min = 0.0  # Adjust according to your LiDAR specs
        scan_msg.range_max = 20.0  # Adjust according to your LiDAR specs
        scan_msg.ranges = scan_data  # Assuming scan_data is a list of range values
        self.publisher_scan.publish(scan_msg)
        
        
    def publish_pcl(self, pcl_data):
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'  # Adjust the frame_id according to your setup
        msg.height = 1  # Point cloud is unorganized
        msg.width = 91*16  # Number of points
        msg.is_bigendian = False
        msg.point_step = 16  # 16 bytes per point (4 floats)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True  # No invalid points


        # Define fields (x, y, z, intensity)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Generate some example points
        # points = np.random.rand(1000, 4).astype(np.float32)
        msg.data = pcl_data.tostring()

        # Publish the message
        self.publisher.publish(msg)

    def cmd_callback(self, msg):
        global steer_cmd,throttle_cmd
        # print("Received new command", msg.steering_angle, msg.acceleration)
        # Process steering and throttle commands received from ackerman_cmd topic
        
        steer_cmd = msg.steering_angle
        throttle_cmd = msg.acceleration 
    
    def euler_to_quaternion(self,roll, pitch, yaw):
        # Calculate the quaternion components
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def publish_image(self,img_):
        # Generate a sample image (you can replace this with your actual image capture code)
        img = np.zeros((img_.shape[0], img_.shape[1], 3), dtype=np.uint8)
        img[:,:,:] = (img_[:,:,::-1]*255).astype(np.uint8)
        # print(img.shape, np.max(img),np.min(img))

        # img.fill(255)  # White image

        # Convert the image to ROS message format
        image_msg = Image_ros()
        image_msg.height = img.shape[0]
        image_msg.width = img.shape[1]
        image_msg.encoding = 'bgr8'
        image_msg.step = img.shape[1] * 3
        image_msg.data = img.tobytes()

        # Publish the image message
        self.publisher_img.publish(image_msg)
        # self.get_logger().info('Published image')

    def publish_data(self):
        # Publish car pose, twist and acceleration
        msg_pose = PoseStamped()
        msg_twist = TwistStamped()
        msg_accel = AccelStamped()
        msg_pose.header.frame_id = 'map'
        msg_twist.header.frame_id = 'map'
        msg_accel.header.frame_id = 'map'
        msg_pose.header.stamp = self.get_clock().now().to_msg()
        msg_twist.header.stamp = self.get_clock().now().to_msg()
        msg_accel.header.stamp = self.get_clock().now().to_msg()
        env_info = self.env.get_steps(self.behavior_name)
        
        # print("Here?")
        if len(env_info[0].obs) == 0 :
            print("No observations")
        
        # print(len(env_info[0].obs))
        # print(env_info[1])
        msg_pose.pose.position.x = float(env_info[0].obs[-1][0][0])
        msg_pose.pose.position.y = float(env_info[0].obs[-1][0][1])
        msg_pose.pose.position.z = float(env_info[0].obs[-1][0][2])
        roll = float(env_info[0].obs[-1][0][3])*math.pi/180.
        pitch = float(env_info[0].obs[-1][0][4])*math.pi/180.
        yaw = float(env_info[0].obs[-1][0][5])*math.pi/180.
        qx,qy,qz,qw = self.euler_to_quaternion(roll,pitch,yaw)
        msg_pose.pose.orientation.x = qx
        msg_pose.pose.orientation.y = qy
        msg_pose.pose.orientation.z = qz
        msg_pose.pose.orientation.w = qw
        msg_twist.twist.linear.x = float(env_info[0].obs[-1][0][6])
        msg_twist.twist.linear.y = float(env_info[0].obs[-1][0][7])
        msg_twist.twist.linear.z = float(env_info[0].obs[-1][0][8])
        msg_twist.twist.angular.x = float(env_info[0].obs[-1][0][9])
        msg_twist.twist.angular.y = float(env_info[0].obs[-1][0][10])
        msg_twist.twist.angular.z = float(env_info[0].obs[-1][0][11])
        msg_accel.accel.linear.x = float(env_info[0].obs[-1][0][12])
        msg_accel.accel.linear.y = float(env_info[0].obs[-1][0][13])
        msg_accel.accel.linear.z = float(env_info[0].obs[-1][0][14])
        
        # Publish transform from map to base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = msg_pose.pose.position.x-500.
        transform.transform.translation.y = msg_pose.pose.position.z - 20.
        transform.transform.translation.z = msg_pose.pose.position.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        ranges = [0.]*91
        for i in range(46):
            ranges[45+i] = 4.*float(env_info[0].obs[self.base_idx][0][4*i+1])
            if i==0 :
                continue
            ranges[45-i] = 4.*float(env_info[0].obs[self.base_idx][0][4*i-1])
        
        # for i in range(18):
        #     print(i,len(env_info[0].obs[i+1][0]))
        pcl = []
        self.publish_image(np.array(env_info[0].obs[0][0]))
        avg_ds = []
        inds = np.array([0,8,9,10,11,12,13,14,15,1,2,3,4,5,6,7]) + 1 # 1-based indexing
        for k in range(16) :
            a = (-15. + 2*k)*math.pi/180.
            j = inds[k]
            # print(a)
            ds = []
            for i in range(46):
                t = (90-2*i)*math.pi/180.
                d = 4.*float(env_info[0].obs[j][0][4*i+1])
                y = d*math.cos(t)*math.cos(a)
                x = d*math.sin(t)*math.cos(a)
                z = d*math.sin(a)
                pcl.append([x,y,z,1.])
                ds.append(d)
                if i == 0:
                    continue
                t = (90+2*i)*math.pi/180.
                d = 4.*float(env_info[0].obs[j][0][4*i-1])
                y = d*math.cos(t)*math.cos(a)
                x = d*math.sin(t)*math.cos(a)
                z = d*math.sin(a)
                pcl.append([x,y,z,1.])
                ds.append(d)
            avg_ds.append(np.mean(ds))

        # print(avg_ds)
        self.publish_pcl(np.array(pcl).astype(np.float32))
        self.publish_scan(ranges)
        # print(steer_cmd,throttle_cmd)
        self.get_logger().info('Published (steer,throttle): '+ str(steer_cmd) + ', ' + str(throttle_cmd))

        actions = ActionTuple(np.array([[steer_cmd,throttle_cmd]]),None)
        self.env.set_actions(self.behavior_name, actions)
       
        self.env.step()
        dt = time.time() - self.curr_time
        if dt < 1./self.fps_target:
            time.sleep(1./self.fps_target - dt)
        self.fps = self.fps + 0.1*(1./(time.time()-self.curr_time) - self.fps)
        self.get_logger().info('Curr FPS: '+ str(self.fps) + ' (' + "{:.2f}".format(self.fps/self.fps_target) + 'x real-time)')
        self.curr_time = time.time()
        self.publisher_pose.publish(msg_pose)
        self.publisher_twist.publish(msg_twist)
        self.publisher_accel.publish(msg_accel)

        

def main(args=None):
    rclpy.init(args=args)
    car_controller = CarController()
    rclpy.spin(car_controller)
    car_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

