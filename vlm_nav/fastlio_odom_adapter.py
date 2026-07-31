"""Convert FAST_LIO camera_init/body odometry to Nav2 odom/base_link."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .geometry import quaternion_matrix


def pose_matrix(xyz, rpy):
    roll, pitch, yaw = [float(value) for value in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    result[:3, 3] = xyz
    return result


def matrix_quaternion(rotation):
    """Convert a 3x3 matrix to x,y,z,w without an extra runtime dependency."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        )
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        return (
            0.25 * s,
            (rotation[0, 1] + rotation[1, 0]) / s,
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[2, 1] - rotation[1, 2]) / s,
        )
    if index == 1:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        return (
            (rotation[0, 1] + rotation[1, 0]) / s,
            0.25 * s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
        )
    s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
    return (
        (rotation[0, 2] + rotation[2, 0]) / s,
        (rotation[1, 2] + rotation[2, 1]) / s,
        0.25 * s,
        (rotation[1, 0] - rotation[0, 1]) / s,
    )


class FastLioOdomAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_odom_adapter")
        defaults = {
            "input_odom_topic": "/Odometry",
            "output_odom_topic": "/fastlio/odom",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "publish_tf": True,
            "base_to_body_xyz": [0.299, -0.02329, 0.32088],
            "base_to_body_rpy": [0.0, 0.0, math.pi],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.t_base_body = pose_matrix(
            self.get_parameter("base_to_body_xyz").value,
            self.get_parameter("base_to_body_rpy").value,
        )
        self.t_body_base = np.linalg.inv(self.t_base_body)
        output_topic = self.get_parameter("output_odom_topic").value
        input_topic = self.get_parameter("input_odom_topic").value
        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, input_topic, self.on_odom, 50)
        self.get_logger().info(
            f"Converting {input_topic} camera_init/body to "
            f"{output_topic} {self.odom_frame}/{self.base_frame}"
        )

    def on_odom(self, message):
        orientation = message.pose.pose.orientation
        source = quaternion_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        source[:3, 3] = [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ]
        converted = source @ self.t_body_base
        qx, qy, qz, qw = matrix_quaternion(converted[:3, :3])
        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = float(converted[0, 3])
        output.pose.pose.position.y = float(converted[1, 3])
        output.pose.pose.position.z = float(converted[2, 3])
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.pose.covariance = message.pose.covariance
        rotation_base_body = self.t_base_body[:3, :3]
        linear_body = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ]
        )
        angular_body = np.array(
            [
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            ]
        )
        angular_base = rotation_base_body @ angular_body
        linear_base = (
            rotation_base_body @ linear_body
            + np.cross(angular_base, -self.t_base_body[:3, 3])
        )
        output.twist.twist.linear.x = float(linear_base[0])
        output.twist.twist.linear.y = float(linear_base[1])
        output.twist.twist.linear.z = float(linear_base[2])
        output.twist.twist.angular.x = float(angular_base[0])
        output.twist.twist.angular.y = float(angular_base[1])
        output.twist.twist.angular.z = float(angular_base[2])
        output.twist.covariance = message.twist.covariance
        self.publisher.publish(output)
        if self.publish_tf:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
