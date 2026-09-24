"""The whole stack against mocks: no robot, no GPU box, no risk.

Real Nav2, the real base adapter and the real orchestrator, driving a base that
exists only in odometry and arms that exist only in ``/joint_states``. The
mission runs start to finish -- goals are planned and reached, the interlock
opens and shuts, the grasp condition fires, the summary prints.

    ros2 launch cerebel_orchestrator desk_test.launch.py mission:=two_station_pick_place

Watch it with:

    ros2 topic echo /orchestrator/status
    ros2 topic echo /orchestrator/base_enable

What this proves: the mission's structure, the station poses and tolerances, the
Nav2 parameters, the interlock, the phase-termination logic, and that a policy
phase starts and stops in the right places. What it does not prove: anything
about the real base's kinematics or odometry, and anything at all about the
policy -- ``mock_policy`` means no inference client is started. It is a
rehearsal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "cerebel_orchestrator"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE)

    return LaunchDescription(
        [
            DeclareLaunchArgument("mission", default_value="two_station_pick_place"),
            DeclareLaunchArgument("kinematics", default_value="holonomic"),
            DeclareLaunchArgument(
                "holonomic",
                default_value="true",
                description=(
                    "make the fake base able to strafe. True by default because "
                    "the real chassis is four-wheel swerve: with this false a "
                    "lateral move_base step integrates zero wheel rpm and runs "
                    "to its timeout"
                ),
            ),
            DeclareLaunchArgument(
                "grasp_after_s",
                default_value="8.0",
                description="when the fake gripper closes, so a grasp condition can fire",
            ),
            DeclareLaunchArgument(
                "mock_policy",
                default_value="true",
                description="keep true unless a real policy server is reachable",
            ),
            Node(
                package=PACKAGE,
                executable="fake_base",
                name="fake_base",
                output="screen",
                parameters=[{"holonomic": LaunchConfiguration("holonomic")}],
            ),
            Node(
                package=PACKAGE,
                executable="fake_arm",
                name="fake_arm",
                output="screen",
                parameters=[{"grasp_after_s": LaunchConfiguration("grasp_after_s")}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "orchestrator.launch.py")
                ),
                launch_arguments={
                    "mission": LaunchConfiguration("mission"),
                    "kinematics": LaunchConfiguration("kinematics"),
                    # Navigation is REAL here -- real Nav2 planning and driving
                    # the fake base is most of what this test is for.
                    "mock_policy": LaunchConfiguration("mock_policy"),
                    "mock_nav": "false",
                    "auto_start": "true",
                    # The fake arms follow commands perfectly, so parking is
                    # exercised here rather than skipped.
                    "enable_park": "true",
                }.items(),
            ),
        ]
    )
