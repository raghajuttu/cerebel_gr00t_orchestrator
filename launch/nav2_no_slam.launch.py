"""Nav2 with no SLAM, no map server and no AMCL.

What is missing here is the point. There is no ``map_server``, so nothing loads
an occupancy grid; no ``amcl``, so nothing corrects the pose; and no
``slam_toolbox``, so nothing builds a map. The global frame is ``odom``, both
costmaps are rolling windows, and the robot's position is whatever the wheels
say. For a base that moves half a metre left and half a metre back, that is
enough -- and it is honest about being dead reckoning. docs/NAVIGATION.md covers
what this costs and what to add first.

The optional ``map -> odom`` static transform is a convenience, not
localisation: it is identity, and exists so RViz and any tool that assumes a
``map`` frame still work. Nothing in the navigation stack depends on it.

    ros2 launch cerebel_orchestrator nav2_no_slam.launch.py kinematics:=holonomic
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "cerebel_orchestrator"

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def _setup(context, *_args, **_kwargs):
    share = get_package_share_directory(PACKAGE)
    kinematics = LaunchConfiguration("kinematics").perform(context)
    params = LaunchConfiguration("params_file").perform(context)
    if not params:
        params = os.path.join(share, "params", f"nav2_{kinematics}.yaml")
    if not os.path.exists(params):
        raise RuntimeError(
            f"Nav2 parameter file not found: {params}. "
            "kinematics must be 'diff_drive' or 'holonomic', or pass params_file."
        )

    use_smoother = LaunchConfiguration("use_velocity_smoother").perform(context) == "true"
    publish_map = LaunchConfiguration("publish_map_frame").perform(context) == "true"
    # With the smoother in the chain the controller's output is renamed, so the
    # base adapter must be told to read cmd_vel_smoothed instead.
    controller_remaps = [("cmd_vel", "cmd_vel_raw")] if use_smoother else []

    nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params],
            remappings=controller_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params],
        ),
    ]

    lifecycle = list(LIFECYCLE_NODES)
    if use_smoother:
        nodes.append(
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[params],
                remappings=[("cmd_vel", "cmd_vel_raw"), ("cmd_vel_smoothed", "cmd_vel")],
            )
        )
        lifecycle.append("velocity_smoother")

    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"node_names": lifecycle},
            ],
        )
    )

    if publish_map:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kinematics",
                default_value="diff_drive",
                description="diff_drive or holonomic -- picks the parameter file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="override the Nav2 parameter file outright",
            ),
            DeclareLaunchArgument(
                "use_velocity_smoother",
                default_value="false",
                description="insert nav2_velocity_smoother between the controller and the base",
            ),
            DeclareLaunchArgument(
                "publish_map_frame",
                default_value="true",
                description="publish an identity map -> odom transform for RViz",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
