from rcl_interfaces.msg import SetParametersResult

import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
#from turtlebot3_perception.landmark_msgs.msg import Landmark
from landmark_msgs.msg import Landmark, LandmarkArray
from ekf_filter.utils import residual

from numpy.linalg import inv

import matplotlib.pyplot as plt
from ekf_filter.ekf import RobotEKF
from ekf_filter.probabilistic_models import sample_velocity_motion_model, velocity_mm_simpy, landmark_sm_simpy, landmark_range_bearing_model

class Filter(Node):

    def __init__(self):
        super().__init__('filter')

        std_lin_vel = 0.1  # [m/s]
        std_ang_vel = np.deg2rad(1.0)  # [rad/s]
        std_range = 0.10  # [m]
        self.sigma_u = np.array([std_lin_vel, std_ang_vel])
        sigma_u_odom = 0

        # Define noise params and Q for landmark sensor model
        std_bearing = np.deg2rad(1.0)  # [rad]
        self.sigma_z = np.array([std_range, std_bearing])
        self.Q_landm = np.diag([std_range**2, std_bearing**2])
        # Define H Jacobian function
        _, self.eval_Ht = landmark_sm_simpy()

        Mt = np.diag([std_lin_vel**2, std_ang_vel**2])
        eval_gux = sample_velocity_motion_model
        _, eval_Gt, eval_Vt = velocity_mm_simpy()

        self.ekf = RobotEKF(
            dim_x = 3, #dimensione mu
            dim_u = 2, #dimensione vel
            eval_gux = eval_gux,
            eval_Gt = eval_Gt,
            eval_Vt = eval_Vt
        )
        self.ekf.mu = np.array([1, 1, 0.3])  # x, y, theta
        self.ekf.Sigma = np.diag([0.1, 0.1, 0.1])
        self.ekf.Mt = Mt


        # #Task 1
        self.landmarks_coord = {
        "id": [11, 12, 13, 21, 22, 23, 31, 32, 33],
        "x": [-1.1, -1.1, -1.1, 0.0, 0.0, 0.0, 1.1, 1.1, 1.1],
        "y": [-1.1, 0.0, 1.1, -1.1, 0.0, 1.1, -1.1, 0.0, 1.1],
       	"z": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }


        self.Odom_sub = self.create_subscription(Odometry, '/odom', self.vel_callback, 10)
        self.Land_sub = self.create_subscription(LandmarkArray, '/landmarks', self.sensor_callback, 10)
        timer_period = 1/20 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher = self.create_publisher(Odometry, '/ekf', 10)

    def vel_callback(self, odom : Odometry):
        self.vel_obtained = odom.twist.twist


    def timer_callback(self):
        vel = np.array([self.vel_obtained.linear.x, self.vel_obtained.angular.z])
        #self.get_logger().info(f'{self.vel_obtained}]')
        self.ekf.predict(u=vel, sigma_u=self.sigma_u, g_extra_args=(1/20,))  #timer callback

    def sensor_callback(self, landmarks : Landmark):
        msg = Odometry()
        eval_hx_landm = landmark_range_bearing_model

        for lmark in landmarks.landmarks:
            index = self.landmarks_coord["id"].index(lmark.id)
            #self.get_logger().info(f'{index}')
            lmarks_coord = [self.landmarks_coord["x"][index], self.landmarks_coord["y"][index]]

            z = np.array([lmark.range, lmark.bearing])
            
                # run the correction step of the EKF
            self.ekf.update(z, eval_hx=eval_hx_landm, eval_Ht=self.eval_Ht, Qt=self.Q_landm, 
                Ht_args=(*self.ekf.mu, *lmarks_coord), # the Ht function requires a flattened array of parameters
                hx_args=(self.ekf.mu, lmarks_coord, self.sigma_z),
                residual=residual, 
                angle_idx=-1)
        msg.pose.pose.position.x = self.ekf.mu[0]
        msg.pose.pose.position.y = self.ekf.mu[1]
        msg.pose.pose.orientation.z = self.ekf.mu[2]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'{self.ekf.mu[2]}')
            
        self.publisher.publish(msg)
        
    
        
def main(args=None):
    rclpy.init(args=args)

    filter = Filter()

    rclpy.spin(filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

