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

import rclpy, math
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool


class Reset(Node):

    def __init__(self):
        super().__init__('reset')

        self.Reset_sub = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Bool, '/reset', 10)

        self.reset_msg = Bool()

    def pose_callback(self, pose : Pose):
        d = math.sqrt((pose.position.x)**2+(pose.position.y)**2)
        if d > 6.0:
            self.reset_msg._data = True
            self.publisher.publish(self.reset_msg)
            self.get_logger().info(f'{self.reset_msg}')

            #self.reset_msg._data = False
        else:
            self.reset_msg._data = False
            self.publisher.publish(self.reset_msg)
            self.get_logger().info(f'{self.reset_msg}')


def main(args=None):
    rclpy.init(args=args)

    reset = Reset()

    rclpy.spin(reset)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reset.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
