from rcl_interfaces.msg import SetParametersResult

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import numpy as np
from a_star.a_star import AStar
import a_star.utils as utils
import a_star.dijkstra as dijkstra
import matplotlib.pyplot as plt
import a_star.pure_pursuit as pursuit
import tf_transformations
from geometry_msgs.msg import Quaternion

from numpy.linalg import inv

class Robot:
    def __init__(self):
        # initialization
        self.pose = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0])

class Pursuit(Node):

    def __init__(self):
        super().__init__('path_tracing')
        #PurePursuit
        initial_state = np.array([[0.0], [0.0], [math.pi/2]])
        self.robot = utils.DifferentialDriveRobot(initial_state)
        # self.robot.x = initial_state
        # self.robot.u = np.array([[0.0], [0.0]])
        self.Lt = 0.2
        self.target_vel = 0.1
        self.kp = 2
        self.path_interpolated = Path()
        self.controller = None

        #Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)

        #Publisher
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(1/15, self.control_loop)

        
    def odom_callback(self, odom : Odometry):
        __, __, yaw = tf_transformations.euler_from_quaternion([
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])

        self.robot.pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        self.robot.vel = np.array([odom.twist.twist.linear.x, odom.twist.twist.angular.z])

    def control_loop(self):
        if self.controller is not None:
            # Calcola la velocità angolare
            lt_parameter = 2
            w = self.controller.angular_velocity()
            a = utils.proportional_control(self.controller.target_velocity(), self.robot.v, kp=self.kp)
            
            # Crea e pubblica il messaggio Twist per inviare i comandi di movimento
            cmd = Twist()
            #Adaptive PurePursuit
            self.controller.Lt = abs(self.robot.v * lt_parameter)
            self.robot.update_state([a, w], 1/15)

            cmd.linear.x = self.robot.v
            cmd.angular.z = self.robot.w

            print(self.controller.Lt)

            self.vel_publisher.publish(cmd)
            self.get_logger().info(f'{cmd}]')

    def path_callback(self, msg : Path):
        path = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        self.path_interpolated = utils.interpolate_waypoints(path, resolution=0.01)
        #self.get_logger().info(f'{self.path_interpolated}]')
        #self.get_logger().info(f"Received path with {len(path)} points.")
        if self.controller == None:
            self.controller = pursuit.PurePursuitController(self.robot, self.path_interpolated, 0, self.Lt, self.target_vel)
        
def main(args=None):
    rclpy.init(args=args)

    pursuit = Pursuit()

    rclpy.spin(pursuit)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

