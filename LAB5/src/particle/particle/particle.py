from rcl_interfaces.msg import SetParametersResult

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from landmark_msgs.msg import Landmark, LandmarkArray
from particle.utils import residual, state_mean, simple_resample, stratified_resample, systematic_resample, residual_resample

from numpy.linalg import inv

from particle.pf import RobotPF
from particle.probabilistic_models import sample_velocity_motion_model, landmark_range_bearing_model

class Filter(Node):

    def __init__(self):
        super().__init__('filter')

        # Probabilistic models parameters
        dim_x = 3
        # First, choose the Motion Model

        # general noise parameters
        std_lin_vel = 0.1  # [m/s]
        std_ang_vel = np.deg2rad(1.0)  # [rad/s]
        self.sigma_u = np.array([std_lin_vel, std_ang_vel])
        sigma_u_odom = 0

        dim_u = 2
        eval_gux = sample_velocity_motion_model

        # Define noise params and Q for landmark sensor model
        std_range = 0.1  # [m]
        std_bearing = np.deg2rad(1.0)  # [rad]
        self.sigma_z = np.array([std_range, std_bearing])

        self.pf = RobotPF(
        dim_x=dim_x,
        dim_u=dim_u,
        eval_gux=eval_gux,
        resampling_fn=simple_resample,
        boundaries=[(-3.0, 3.0), (-3.0, 3.0), (-np.pi, np.pi)],
        N=1000,
        )

        #self.pf.mu = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.pf.Sigma = np.diag([0.1, 0.1, 0.1])
        self.pf.initialize_particles()

        self.eval_hx_landm = landmark_range_bearing_model


        self.landmarks_coord = {
        "id": [11, 12, 13, 21, 22, 23, 31, 32, 33],
        "x": [-1.1, -1.1, -1.1, 0.0, 0.0, 0.0, 1.1, 1.1, 1.1],
        "y": [-1.1, 0.0, 1.1, -1.1, 0.0, 1.1, -1.1, 0.0, 1.1],
        "z": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }

        self.Twist_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.Land_sub = self.create_subscription(LandmarkArray, '/landmarks', self.sensor_callback, 10)
        timer_period = 1/20 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher = self.create_publisher(Odometry, '/pf', 10)
        self.msg = Odometry()
        self.vel_obtained = Twist()

    def vel_callback(self, twist : Twist):
        self.vel_obtained = twist

    def timer_callback(self):
        cmd_vel = np.array([self.vel_obtained.linear.x, self.vel_obtained.angular.z])
        #self.get_logger().info(f'{cmd_vel}]')
        self.pf.predict(u=cmd_vel, sigma_u=self.sigma_u, g_extra_args=(1/20,))

    def sensor_callback(self, landmarks : Landmark):
        self.get_logger().info(f'{"ok"}')
        for lmark in landmarks.landmarks:
            index = self.landmarks_coord["id"].index(lmark.id)
            #self.get_logger().info(f'{index}')
            lmarks_coord = [self.landmarks_coord["x"][index], self.landmarks_coord["y"][index]]

            z = np.array([lmark.range, lmark.bearing])
            self.pf.update(z, self.sigma_z, eval_hx=self.eval_hx_landm, hx_args=(lmarks_coord, self.sigma_z))

        self.pf.normalize_weights()

        neff = self.pf.neff()

        if neff < self.pf.N / 2:
            self.pf.resampling(
                resampling_fn=self.pf.resampling_fn,  # simple, residual, stratified, systematic
                resampling_args=(self.pf.weights,),  # tuple: only pf.weights if using pre-defined functions
            )
            assert np.allclose(self.pf.weights, 1 / self.pf.N)

        # estimate robot mean and covariance from particles
        self.pf.estimate(mean_fn=state_mean, residual_fn=residual, angle_idx=2)

        self.msg.pose.pose.position.x = self.pf.mu[0]
        self.msg.pose.pose.position.y = self.pf.mu[1]
        self.msg.pose.pose.orientation.z = self.pf.mu[2]
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'{self.pf.mu}')
            
        self.publisher.publish(self.msg)
        
    
        
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

