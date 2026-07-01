from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joint_position_tolerance = LaunchConfiguration('joint_position_tolerance')
    place_tcp_z = LaunchConfiguration('place_tcp_z')
    marker_size = LaunchConfiguration('marker_size')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_position_tolerance',
            default_value='0.2',
            description='Joint position tolerance used to decide when the arm reached its target'
        ),

        DeclareLaunchArgument(
            'place_tcp_z',
            default_value='0.040',
            description='TCP z height used when placing the object'
        ),

        DeclareLaunchArgument(
            'marker_size',
            default_value='0.040',
            description='ArUco marker side length in meters'
        ),

        Node(
            package='piperx_perception',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[
                {'marker_size': marker_size}
            ]
        ),

        Node(
            package='piperx_control',
            executable='pick_place_controller',
            name='pick_place_controller',
            output='screen',
            parameters=[
                {
                'joint_position_tolerance': joint_position_tolerance,
                'place_tcp_z': place_tcp_z
                }
            ]
        )
    ])