from rcl_interfaces.msg import SetParametersResult

import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Declare a parameter
        self.declare_parameter('linear', 0.5)
        self.speed = self.get_parameter('linear').get_parameter_value().double_value

        self.declare_parameter('angular', 0.2)
        self.speed_angular = self.get_parameter('angular').get_parameter_value().double_value

        #Node
        self.Scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.Odom_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer_period = 1/10
        self.timer = self.create_timer(self.timer_period, self.move_callback)

        self.obstacle = False
        self.vel = Twist()
        self.angle = Pose()
        self.is_rotating = False
        self.complete = True

    def scan_callback(self, laserscan : LaserScan):

        x = np.array(laserscan.ranges[-15:]+laserscan.ranges[:15])
        if np.min(x) < 1.0:
            self.obstacle = True
        else:
            self.obstacle = False

        if self.complete:
            if laserscan.ranges[90] > laserscan.ranges[270]:
                self.is_rotating = True #Sx
            else:
                self.is_rotating = False #Dx

    def pose_callback(self, odometry : Odometry):
        self.vel_obtained = odometry.twist.twist
        self.angle = odometry.pose.pose.orientation
        quat = [self.angle.x, self.angle.y, self.angle.z, self.angle.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat)
        if self.complete:
            _, _, self.yaw_control = tf_transformations.euler_from_quaternion(quat)
            

    def turn_left(self):
        if (self.yaw_control + math.pi/2) > math.pi:
            return self.yaw_control + math.pi/2 - 2*math.pi
        else:
            return (self.yaw_control + math.pi/2)

    def turn_right(self):
        if (self.yaw_control - math.pi/2) < -math.pi:
            return self.yaw_control - math.pi/2 + 2*math.pi
        else:
            return (self.yaw_control - math.pi/2)


    def move_callback(self):
        msg = Twist()
        left = self.turn_left()
        right = self.turn_right()
        if not self.obstacle and self.complete:
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            self.get_logger().info(f'{self.yaw}')
        else:
            if self.is_rotating:
                msg.linear.x = 0.0
                msg.angular.z = self.speed_angular
                if abs(self.yaw - left) > self.speed_angular * self.timer_period * 0.99:
                    self.complete = False
                else: 
                    self.complete = True
                    msg.angular.z = 0.0
            else:
                self.get_logger().info(f'{right}')
                msg.linear.x = 0.0
                msg.angular.z = -self.speed_angular
                if abs(self.yaw - right) > self.speed_angular * self.timer_period * 0.99:
                    self.complete = False
                else: 
                    self.complete = True
                    msg.angular.z = 0.0
            self.get_logger().info(f'{self.yaw}')
            self.get_logger().info(f'{self.yaw_control}')
        self.publisher.publish(msg)
        self.get_logger().info(f'{msg}')
        

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



