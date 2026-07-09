#!/usr/bin/env python3

import cv2
import rclpy
import tf2_ros
import numpy as np
import cv2.aruco as aruco

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_matrix


class CharucoCalibrationDetector(Node):
    def __init__(self):
        super().__init__("charuco_calibration_detector")

        self.bridge = CvBridge()

        self.declare_parameter("squares_x", 7)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length", 0.037)   # 3.7 cm
        self.declare_parameter("marker_length", 0.027)   # 2.7 cm, marker only

        self.squares_x = self.get_parameter("squares_x").value
        self.squares_y = self.get_parameter("squares_y").value
        self.square_length = self.get_parameter("square_length").value
        self.marker_length = self.get_parameter("marker_length").value

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        self.board = aruco.CharucoBoard_create(
            self.squares_x,
            self.squares_y,
            self.square_length,
            self.marker_length,
            self.aruco_dict
        )

        self.camera_matrix = None
        self.dist_coeffs = None
        self.printed_camera_info = False

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            1
        )

        self.get_logger().info("ChArUco calibration detector started.")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

        if not self.printed_camera_info:
            self.get_logger().info(
                f"Received camera intrinsics:\n{self.camera_matrix}"
            )
            self.get_logger().info(
                f"Camera info frame_id: {msg.header.frame_id}"
            )
            self.printed_camera_info = True

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None or len(ids) == 0:
            return

        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs
        )

        if charuco_ids is None or charuco_corners is None:
            return

        if len(charuco_ids) < 16:
            self.get_logger().warn(
                f"Only {len(charuco_ids)} ChArUco corners detected; need more."
            )
            return

        valid, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            self.camera_matrix,
            self.dist_coeffs,
            None,
            None
        )

        if not valid:
            return

        self.broadcast_charuco_tf(rvec, tvec, msg, len(charuco_ids))

    def broadcast_charuco_tf(self, rvec, tvec, image_msg, num_corners):
        R_camera_target, _ = cv2.Rodrigues(rvec)

        T_camera_target = np.eye(4)
        T_camera_target[0:3, 0:3] = R_camera_target
        T_camera_target[0, 3] = float(tvec[0])
        T_camera_target[1, 3] = float(tvec[1])
        T_camera_target[2, 3] = float(tvec[2])

        quat = quaternion_from_matrix(T_camera_target)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = image_msg.header.stamp
        tf_msg.header.frame_id = image_msg.header.frame_id
        tf_msg.child_frame_id = "charuco_calibration_target"

        tf_msg.transform.translation.x = float(T_camera_target[0, 3])
        tf_msg.transform.translation.y = float(T_camera_target[1, 3])
        tf_msg.transform.translation.z = float(T_camera_target[2, 3])

        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f"Published camera -> charuco_calibration_target TF using {num_corners} corners."
        )


def main(args=None):
    rclpy.init(args=args)
    node = CharucoCalibrationDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()