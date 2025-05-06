# Copyright 2024 Juan Carlos Manzanares Serrano
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

from ros2cli.verb import VerbExtension
from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.strategy import add_arguments
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import math


class SetPoseVerb(VerbExtension):
    """Set pose."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '-x',
            help='Coordinate x',)
        parser.add_argument(
            '-y',
            help='Coordinate y',)
        parser.add_argument(
            '-yaw',
            help='Rotation yaw',)
        parser.add_argument(
            'x_coordinate',
            help='Coordinate x',)
        parser.add_argument(
            'y_coordinate',
            help='Coordinate y',)
        parser.add_argument(
            'yaw_rotation',
            help='Rotation yaw',)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node.get_logger().info('Set pose')
            publisher = node.create_publisher(
                PoseWithCovarianceStamped, 'initialpose', 10)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"

            if args.x:
                msg.pose.pose.position.x = float(args.x)
            else:
                msg.pose.pose.position.x = float(args.x_coordinate)

            if args.y:
                msg.pose.pose.position.y = float(args.y)
            else:
                msg.pose.pose.position.y = float(args.y_coordinate)

            if args.yaw:
                quaternion = Quaternion()
                quaternion = self.quaternion_from_euler(0, 0, float(args.yaw))
                msg.pose.pose.orientation = quaternion
            else:
                quaternion = Quaternion()
                quaternion = self.quaternion_from_euler(
                    0, 0, float(args.yaw_rotation))
                msg.pose.pose.orientation = quaternion

            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891945200942

            publisher.publish(msg)

        return 0
