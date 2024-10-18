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

from geometry_msgs.msg import Pose, Twist


class Localization(Node):

    def __init__(self):
        super().__init__('localization')
        self.Twist_sub = self.create_subscription(Twist, '/cmd_topic', self.twist_callback, 10)
        self.publisher = self.create_publisher(Pose, '/pose', 10)
        
        self.pose_msg = Pose()

    def twist_callback(self, twist : Twist):
        self.pose_msg.position.x = self.pose_msg.position.x + twist.linear.x
        self.pose_msg.position.y = self.pose_msg.position.y + twist.linear.y
        self.publisher.publish(self.pose_msg)
        self.get_logger().info(f'{self.pose_msg}')



def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
