# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.N = 1
        self.t = 0

        self.first = 0

    def timer_callback(self):
        msg = Twist()

        if(self.first == 0):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            self.get_logger().info(f'Velocity: {msg.linear}')
            self.publisher_.publish(msg)
            self.first += 1
        
        elif self.N%4 == 1:
            if self.t < self.N-1:
                msg.linear.x = 1.0
                self.get_logger().info(f'Velocity x: {msg.linear}')
                self.publisher_.publish(msg)
                self.t = self.t + 1
            else:
                msg.linear.x = 1.0
                self.get_logger().info(f'Velocity x: {msg.linear}')
                self.publisher_.publish(msg)
                self.N = self.N +1
                self.t = 0

        elif self.N%4 == 2:
            if self.t < self.N - 1:
                msg.linear.y = 1.0
                self.get_logger().info(f'Velocity y: {msg.linear}')
                self.publisher_.publish(msg)
                self.t = self.t + 1
            else:
                msg.linear.y = 1.0
                self.get_logger().info(f'Velocity y: {msg.linear}')
                self.publisher_.publish(msg)
                self.N = self.N +1
                self.t = 0


        elif self.N%4 == 3:
            if self.t < self.N - 1:
                msg.linear.x = -1.0
                self.get_logger().info(f'Velocity x: {msg.linear}')
                self.publisher_.publish(msg)
                self.t = self.t + 1
            else:
                msg.linear.x = -1.0
                self.get_logger().info(f'Velocity x: {msg.linear}')
                self.publisher_.publish(msg)
                self.N = self.N + 1
                self.t = 0

        elif self.N%4 == 0:
            if self.t < self.N -1:
                msg.linear.y = -1.0
                self.get_logger().info(f'Velocity y: {msg.linear}')
                self.publisher_.publish(msg)
                self.t = self.t + 1
            else:
                msg.linear.y = -1.0
                self.get_logger().info(f'Velocity y: {msg.linear}')
                self.publisher_.publish(msg)
                self.N = self.N +1
                self.t = 0


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
