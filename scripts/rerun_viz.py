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
from rclpy.time import (
    Duration,
    Time,
)
from sensor_msgs.msg import (
    CameraInfo,
    Image,
)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class O3DERosProjectSubscriber(Node):
    def __init__(self):
        super().__init__("o3de_ros_project_subscriber")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
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

        self.parent_frame = "car/car"

        # Log a bounding box as a visual placeholder for the map
        rr.log(
            "map/box",
            rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            timeless=True,
        )

        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/car/color/camera_info",
            self.cam_info_callback,
            qos_profile=qos_profile,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/car/color/image_color",
            self.image_callback,
            qos_profile=qos_profile,
            callback_group=self.callback_group,
        )

    def log_tf_as_transform3d(self, child_frame: str, time: Time) -> None:
        """
        Helper to look up a transform with tf and log using `log_transform3d`.

        Note: we do the lookup on the client side instead of re-logging the raw transforms until
        Rerun has support for Derived Transforms [#1533](https://github.com/rerun-io/rerun/issues/1533)
        """

        # Do the TF lookup to get transform from child (source) -> parent (target)
        try:
            tf = self.tf_buffer.lookup_transform(self.parent_frame, child_frame, time, timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(child_frame, rr.Transform3D(translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))
        except TransformException as ex:
            print(f"Failed to get transform: {ex}")

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

    def image_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        rr.log(img.header.frame_id, rr.Image(self.cv_bridge.imgmsg_to_cv2(img)))
        self.log_tf_as_transform3d(img.header.frame_id, time)


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
