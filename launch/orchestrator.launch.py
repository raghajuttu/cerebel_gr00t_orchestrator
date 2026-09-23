"""The orchestrator and the base adapter, with Nav2 optionally underneath.

    # rehearse a mission with nothing moving
    ros2 launch cerebel_orchestrator orchestrator.launch.py \\
        mission:=two_station_pick_place dry_run:=true

    # base only, on hardware, arms untouched
    ros2 launch cerebel_orchestrator orchestrator.launch.py \\
        mission:=nav_only auto_start:=false

    # the real thing, once every step has been proven on its own
    ros2 launch cerebel_orchestrator orchestrator.launch.py \\
        mission:=two_station_pick_place enable_park:=true auto_start:=false

``mission`` takes either a bare name from this package's ``missions/`` directory
or an absolute path. Nothing starts moving until ``~/start`` is called unless
``auto_start:=true``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "cerebel_orchestrator"


def _resolve_mission(share: str, value: str) -> str:
    if not value:
        raise RuntimeError("mission: is required (a name from missions/, or a path)")
    for candidate in (
        value,
        os.path.join(share, "missions", value),
        os.path.join(share, "missions", f"{value}.yaml"),
    ):
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
    available = sorted(
        name[: -len(".yaml")]
        for name in os.listdir(os.path.join(share, "missions"))
        if name.endswith(".yaml")
    )
    raise RuntimeError(f"mission {value!r} not found. Installed missions: {available}")


def _setup(context, *_args, **_kwargs):
    share = get_package_share_directory(PACKAGE)
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    mission = _resolve_mission(share, get("mission"))
    params = get("params_file") or os.path.join(share, "params", "orchestrator.yaml")
    park_poses = get("park_poses_file")
    if not park_poses and get("enable_park") == "true":
        park_poses = os.path.join(share, "params", "park_poses.yaml")
    # check_arms steps need envelopes; default to the shipped file so a mission
    # that uses one does not fail at startup for want of a launch argument.
    envelopes = get("arm_envelopes_file") or os.path.join(
        share, "params", "arm_envelopes.yaml"
    )

    # dry_run is the convenience that sets both mocks; either can also be set
    # on its own, which is how the desk test keeps navigation real.
    dry_run = get("dry_run") == "true"
    overrides = {
        "mission_file": mission,
        "auto_start": get("auto_start") == "true",
        "mock_policy": dry_run or get("mock_policy") == "true",
        "mock_nav": dry_run or get("mock_nav") == "true",
        "enable_park": get("enable_park") == "true",
        "arm_envelopes_file": envelopes,
    }
    if park_poses:
        overrides["park_poses_file"] = park_poses

    nodes = [
        Node(
            package=PACKAGE,
            executable="base_adapter",
            name="base_adapter",
            output="screen",
            parameters=[params],
        ),
        Node(
            package=PACKAGE,
            executable="orchestrator",
            name="orchestrator",
            output="screen",
            emulate_tty=True,
            parameters=[params, overrides],
        ),
    ]
    return nodes


def generate_launch_description() -> LaunchDescription:
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(PACKAGE), "launch", "nav2_no_slam.launch.py"
            )
        ),
        launch_arguments={
            "kinematics": LaunchConfiguration("kinematics"),
            "use_velocity_smoother": LaunchConfiguration("use_velocity_smoother"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mission", description="mission name or path"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("park_poses_file", default_value=""),
            DeclareLaunchArgument("arm_envelopes_file", default_value=""),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="start the mission on launch instead of waiting for ~/start",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="shorthand for mock_policy:=true mock_nav:=true",
            ),
            DeclareLaunchArgument(
                "mock_policy",
                default_value="false",
                description="run no inference client; the arms stay where they are",
            ),
            DeclareLaunchArgument(
                "mock_nav",
                default_value="false",
                description="send no Nav2 goal; a navigate step just succeeds",
            ),
            DeclareLaunchArgument(
                "enable_park",
                default_value="false",
                description="let park_arms actually move the arms",
            ),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("kinematics", default_value="diff_drive"),
            DeclareLaunchArgument("use_velocity_smoother", default_value="false"),
            nav2,
            OpaqueFunction(function=_setup),
        ]
    )
