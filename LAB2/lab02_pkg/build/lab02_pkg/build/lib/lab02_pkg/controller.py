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
        self.declare_parameter('linear', 0.22)
        self.speed = self.get_parameter('linear').get_parameter_value().double_value

        self.declare_parameter('angular', 1.5)
        self.speed_angular = self.get_parameter('angular').get_parameter_value().double_value

        #Node
        self.Scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.Odom_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 1/10
        self.timer = self.create_timer(timer_period, self.move_callback)

        self.obstacle = False
        self.vel = Twist()
        self.angle = Pose()
        self.is_rotating = False

    # def rotate_90_degrees(self):
    #     duration = 90 / 360 * (2 * 3.14159) / self.rotation_speed  # Convert 90 degrees to radians
    #     self.is_rotating = True
    #     self.start_time = self.get_clock().now()
    #     twist = Twist()
    #     twist.angular.z = self.rotation_speed  # Rotate counter-clockwise
    #     self.publisher_.publish(twist)
    #     self.get_logger().info('Rotating 90 degrees.')

    #     # Start a timer to stop the rotation after the calculated duration
    #     self.timer = self.create_timer(duration, self.stop_robot)

    def scan_callback(self, laserscan : LaserScan):
        x = np.array(laserscan.ranges[-10:10])
        if np.min(x) < 1.0:
            self.obstacle = True
        else:
            self.obstacle = False

        if laserscan.ranges[90] > laserscan.ranges[270]:
            self.is_rotating = True #Sx
        else:
            self.is_rotating = False #Dx

    def pose_callback(self, odometry : Odometry):
        self.vel_obtained = odometry.twist.twist
        self.angle = odometry.pose.pose.orientation
        quat = [self.angle.x, self.angle.y, self.angle.z, self.angle.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat)

    def move_callback(self):
        msg = Twist()
        if not self.obstacle:
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            self.get_logger().info(f'{self.yaw}')
        else:
            if self.is_rotating:
                msg.linear.x = 0.0
                msg.angular.z = self.speed_angular
            else:
                msg.linear.x = 0.0
                msg.angular.z = -self.speed_angular
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



