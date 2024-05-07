import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import pygame
import math

class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(AckermannDrive, '/cmd', 10)
        self.ackermann_msg = AckermannDrive()

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_ackermann_drive)

        # Initialize Pygame
        pygame.init()
        pygame.display.set_mode((100, 100))  # Create a window to capture events

    def publish_ackermann_drive(self):
        # Reset values
        # self.ackermann_msg.acceleration = 0.0
        # self.ackermann_msg.steering_angle = 0.0

        # Handle keyboard events
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.ackermann_msg.acceleration = 0.5  # Set your desired speed here
                elif event.key == pygame.K_DOWN:
                    self.ackermann_msg.acceleration = -0.5  # Set your desired speed here
                elif event.key == pygame.K_LEFT:
                    self.ackermann_msg.steering_angle = -math.pi / 8  # Set your desired steering angle here
                elif event.key == pygame.K_RIGHT:
                    self.ackermann_msg.steering_angle = math.pi / 8  # Set your desired steering angle here
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_UP:
                    self.ackermann_msg.acceleration = 0.  # Set your desired speed here
                elif event.key == pygame.K_DOWN:
                    self.ackermann_msg.acceleration = 0.  # Set your desired speed here
                elif event.key == pygame.K_LEFT:
                    self.ackermann_msg.steering_angle = 0.  # Set your desired steering angle here
                elif event.key == pygame.K_RIGHT:
                    self.ackermann_msg.steering_angle = 0.  # Set your desired steering angle here

        # print("Publishing: acceleration={}, steering_angle={}".format(self.ackermann_msg.acceleration, self.ackermann_msg.steering_angle))
        self.publisher_.publish(self.ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()