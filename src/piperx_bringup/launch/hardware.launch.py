from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_port_arg = DeclareLaunchArgument("can_port", default_value="can0")
    speed_percent_arg = DeclareLaunchArgument("speed_percent", default_value="10")
    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.1058, 0.0, 0.0, 0.0]",
    )

    agilex_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "arm_type": "piper_x",
            "effector_type": "agx_gripper",
            "can_port": LaunchConfiguration("can_port"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
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
            "-0.06500361859798431",
            "-0.010299943286180496",
            "0.023796094581484795",
            "-1.5273690223693848e-07",
            "-0.5728791356086731",
            "-2.2351741790771484e-08",
            "0.8196398615837097",
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
        can_port_arg,
        speed_percent_arg,
        tcp_offset_arg,
        agilex_moveit_launch,
        realsense_launch,
        camera_static_tf,
        set_robot_state_publisher_frequency,
    ])