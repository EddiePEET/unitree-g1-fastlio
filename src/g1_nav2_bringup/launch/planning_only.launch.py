import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("g1_nav2_bringup"),
        "config",
        "nav2_params.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_nodes = [
        "controller_server",
        "planner_server",
        "recoveries_server",
        "bt_navigator",
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),

        # FAST-LIO body frame follows the upside-down MID360 mounting.
        # This fixed transform creates a planar navigation frame whose Z axis points up.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_nav_base",
            output="screen",
            arguments=[
                "0", "0", "0",
                "1", "0", "0", "0",
                "body", "nav_base",
            ],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_recoveries",
            executable="recoveries_server",
            name="recoveries_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": nav2_nodes,
            }],
        ),
    ])
