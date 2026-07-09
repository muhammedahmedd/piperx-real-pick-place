from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration_name = "piperx_d435_eye_in_hand"

    calibration_type = "eye_in_hand"

    robot_base_frame = "base_link"
    robot_effector_frame = "gripper_base"

    tracking_base_frame = "camera_color_optical_frame"
    tracking_marker_frame = "charuco_calibration_target"

    handeye_server = Node(
        package="easy_handeye2",
        executable="handeye_server",
        name="handeye_server",
        parameters=[{
            "name": calibration_name,
            "calibration_type": calibration_type,
            "tracking_base_frame": tracking_base_frame,
            "tracking_marker_frame": tracking_marker_frame,
            "robot_base_frame": robot_base_frame,
            "robot_effector_frame": robot_effector_frame,
        }],
        output="screen",
    )

    handeye_rqt_calibrator = Node(
        package="easy_handeye2",
        executable="rqt_calibrator.py",
        name="handeye_rqt_calibrator",
        parameters=[{
            "name": calibration_name,
            "calibration_type": calibration_type,
            "tracking_base_frame": tracking_base_frame,
            "tracking_marker_frame": tracking_marker_frame,
            "robot_base_frame": robot_base_frame,
            "robot_effector_frame": robot_effector_frame,
        }],
        output="screen",
    )

    return LaunchDescription([
        handeye_server,
        handeye_rqt_calibrator,
    ])