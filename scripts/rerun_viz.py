#!/usr/bin/env python3
"""
Lucas Walter
April 2024

Send images and point cloud data into Rerun (rerun.io)

adapted from rerun/examples/python/ros_node
"""

import argparse

import cv_bridge
import rclpy
import rerun as rr
from image_geometry import PinholeCameraModel
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import (
    CameraInfo,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class O3DERosProjectSubscriber(Node):
    def __init__(self):
        super().__init__("o3de_ros_project_subscriber")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Assorted helpers for data conversions
        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        # Log a bounding box as a visual placeholder for the map
        rr.log(
            "map/box",
            rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            timeless=True,
        )

        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/robot/color/camera_info",
            self.cam_info_callback,
            qos_profile=qos_profile,
            callback_group=self.callback_group,
        )

    def cam_info_callback(self, info: CameraInfo) -> None:
        """Log a `CameraInfo` with `log_pinhole`."""
        time = Time.from_msg(info.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        self.model.fromCameraInfo(info)

        rr.log(
            "map/robot/camera/img",
            rr.Pinhole(
                resolution=[self.model.width, self.model.height],
                image_from_camera=self.model.intrinsicMatrix(),
            ),
        )


def main():
    parser = argparse.ArgumentParser(description="Simple example of a ROS node that republishes to Rerun.")
    rr.script_add_args(parser)
    args, unknownargs = parser.parse_known_args()
    rr.script_setup(args, "rerun_example_ros_node")

    # Any remaining args go to rclpy
    rclpy.init(args=unknownargs)

    o3de_subscriber = O3DERosProjectSubscriber()

    # Use the MultiThreadedExecutor so that calls to `lookup_transform` don't block the other threads
    rclpy.spin(o3de_subscriber, executor=rclpy.executors.MultiThreadedExecutor())

    o3de_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
