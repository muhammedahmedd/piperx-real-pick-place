from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agilex_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "can_port": "can0",
            "arm_type": "piper_x",
            "effector_type": "agx_gripper",
            "speed_percent": "10",
            "tcp_offset": "[0.0, 0.0, 0.1058, 0.0, 0.0, 0.0]",
        }.items(),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        )
    )

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gripper_to_camera_tf",
        arguments=[
            "-0.07309607416391373",
            "-0.011500004678964615",
            "0.03804999962449074",
            "3.725290298461914e-09",
            "-0.5728673934936523",
            "-2.60770320892334e-08",
            "0.8196479678153992",
            "gripper_base",
            "camera_link",
        ],
        output="screen",
    )

    set_robot_state_publisher_frequency = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "param",
                    "set",
                    "/robot_state_publisher",
                    "publish_frequency",
                    "100.0",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        agilex_moveit_launch,
        realsense_launch,
        camera_static_tf,
        set_robot_state_publisher_frequency,
    ])