"""Bring the arms to a known pose before the base moves.

Between a policy phase and a navigate phase the arms are wherever the policy left
them, holding that pose because ``forward_position_controller`` latches its last
command. Driving a mobile base with two 7-DOF arms extended is how you shear a
gripper off on a door frame, so every navigate step in a mission should be
preceded by ``park_arms``.

**This node commands the arms directly.** It is the only part of the
orchestrator that does, and it is disabled by default (``enable_park:=false``):
with parking disabled the step logs what it would have done and reports success,
so a mission can be rehearsed end to end before anything moves. Enable it only
once the poses in ``params/park_poses.yaml`` have been checked on the real robot,
joint by joint, at a crawl.

The motion is a pure linear ramp in joint space from the measured pose to the
target, at ``max_joint_speed`` rad/s, published at ``rate_hz``. There is no
collision checking and no planning: a park pose has to be reachable from
anywhere the policy can leave the arm by moving every joint monotonically. Keep
the poses conservative -- arms tucked, elbows low -- and keep the speed low.

Standalone use, for checking a pose before trusting it in a mission:

    ros2 run cerebel_orchestrator arm_park --ros-args \\
        -p enable_park:=true -p profile:=travel -p max_joint_speed:=0.15
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
import yaml
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .joints import ARM_SLICE, reorder_by_name

# The joint-state subscription must be BEST_EFFORT. Against the joint state
# broadcaster's RELIABLE + TRANSIENT_LOCAL publisher, a RELIABLE subscriber
# latches the initial all-zero sample and never tracks motion -- the inference
# client lost a whole test session to this (adibot_gr00t_client CHANGELOG
# v0.2.0). A ramp that starts from a false all-zero pose is far worse than one
# that never starts.
JOINT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Used when no park_poses file is given: both arms at zero, grippers open.
# Zero is a *safe-to-publish placeholder*, not a good travel pose -- on the
# OpenArm it is the arm straight out. Override it before enabling parking.
FALLBACK_POSE = {
    "left_arm": [0.0] * 7,
    "right_arm": [0.0] * 7,
    "left_gripper": 0.04,
    "right_gripper": 0.04,
}


class ParkPose:
    """One named park profile: 7 joints per arm plus a finger position each."""

    def __init__(self, name: str, body: Dict) -> None:
        self.name = name
        self.left_arm = self._arm(body, "left_arm", name)
        self.right_arm = self._arm(body, "right_arm", name)
        self.left_gripper = float(body.get("left_gripper", FALLBACK_POSE["left_gripper"]))
        self.right_gripper = float(body.get("right_gripper", FALLBACK_POSE["right_gripper"]))

    @staticmethod
    def _arm(body: Dict, key: str, profile: str) -> List[float]:
        values = body.get(key, FALLBACK_POSE[key])
        if len(values) != 7:
            raise ValueError(
                f"park profile {profile!r}: {key} must have 7 values, got {len(values)}"
            )
        return [float(v) for v in values]

    def as_canonical(self) -> List[float]:
        return (
            list(self.left_arm)
            + [self.left_gripper]
            + list(self.right_arm)
            + [self.right_gripper]
        )


def load_park_poses(path: str) -> Dict[str, ParkPose]:
    """Read ``params/park_poses.yaml``; fall back to a single 'travel' profile."""
    if not path:
        return {"travel": ParkPose("travel", dict(FALLBACK_POSE))}
    with open(os.path.expanduser(path), "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    profiles = raw.get("profiles", raw)
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError(f"{path}: no park profiles found")
    return {name: ParkPose(name, body or {}) for name, body in profiles.items()}


class ArmParker:
    """Poll-shaped ramp to a park pose, driven by the orchestrator's timer.

    ``start`` latches the current measured pose as the ramp's origin; ``poll``
    publishes one step per call and returns None until the pose is reached.
    """

    def __init__(
        self,
        node: Node,
        poses: Dict[str, ParkPose],
        enable: bool,
        max_joint_speed: float = 0.15,
        gripper_max_effort: float = 10.0,
        tolerance: float = 0.01,
        left_topic: str = "/left_forward_position_controller/commands",
        right_topic: str = "/right_forward_position_controller/commands",
        left_gripper_action: str = "/left_gripper_controller/gripper_cmd",
        right_gripper_action: str = "/right_gripper_controller/gripper_cmd",
        command_grippers: bool = True,
    ) -> None:
        self.node = node
        self.log = node.get_logger()
        self.poses = poses
        self.enable = enable
        self.max_joint_speed = max_joint_speed
        self.gripper_max_effort = gripper_max_effort
        self.tolerance = tolerance
        self.command_grippers = command_grippers

        self._left_pub = node.create_publisher(Float64MultiArray, left_topic, 10)
        self._right_pub = node.create_publisher(Float64MultiArray, right_topic, 10)
        self._grippers = {
            "left": ActionClient(node, GripperCommand, left_gripper_action),
            "right": ActionClient(node, GripperCommand, right_gripper_action),
        }
        node.create_subscription(
            JointState, "/joint_states", self._on_joint_states, JOINT_STATE_QOS
        )

        self.positions: Optional[List[float]] = None
        self._target: Optional[ParkPose] = None
        self._origin: Optional[List[float]] = None
        self._start_time: Optional[float] = None
        self._duration: float = 0.0
        self._gripper_sent = False

    # -- state ---------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        ordered = reorder_by_name(msg.name, msg.position)
        if ordered is not None:
            self.positions = ordered

    @property
    def has_state(self) -> bool:
        return self.positions is not None

    # -- the ramp ------------------------------------------------------------

    def start(self, profile: str) -> Optional[str]:
        """Begin a park. Returns an error string if it cannot start."""
        pose = self.poses.get(profile)
        if pose is None:
            return f"no park profile named {profile!r} (have {sorted(self.poses)})"
        if not self.enable:
            self.log.warning(
                f"park [{profile}] SKIPPED: enable_park is false. The arms stay "
                "where the policy left them."
            )
            self._target = None
            return None
        if self.positions is None:
            return "no /joint_states yet -- cannot ramp from an unknown pose"

        self._target = pose
        self._origin = list(self.positions)
        target = pose.as_canonical()
        # Only the 14 arm joints are ramped; the fingers are an action goal.
        travel = max(
            abs(target[i] - self._origin[i])
            for side in ("left", "right")
            for i in range(ARM_SLICE[side].start, ARM_SLICE[side].stop)
        )
        self._duration = travel / max(self.max_joint_speed, 1e-6)
        self._start_time = self._now()
        self._gripper_sent = False
        self.log.info(
            f"park [{profile}]: largest joint move {travel:.3f} rad, "
            f"ramping over {self._duration:.1f}s at {self.max_joint_speed} rad/s"
        )
        return None

    def poll(self) -> Optional[Tuple[bool, str]]:
        """None while ramping; (ok, reason) when the park is over."""
        if self._target is None:
            return (True, "park skipped (enable_park is false)")
        if self._start_time is None:
            return (False, "park polled before it was started")

        fraction = 1.0 if self._duration <= 0 else min(
            1.0, (self._now() - self._start_time) / self._duration
        )
        target = self._target.as_canonical()
        command = [
            origin + (goal - origin) * fraction
            for origin, goal in zip(self._origin or target, target)
        ]
        self._publish_arms(command)

        if fraction >= 1.0 and not self._gripper_sent:
            self._publish_grippers(self._target)
            self._gripper_sent = True

        if fraction < 1.0:
            return None

        error = self._pose_error(target)
        if error is None:
            return (True, "ramp complete (no joint_states to verify against)")
        if error <= self.tolerance:
            return (True, f"parked [{self._target.name}], worst joint error {error:.4f} rad")
        # The controller may still be catching up; give it the tolerance window
        # rather than declaring a failure on the first tick past the ramp.
        if self._now() - self._start_time > self._duration + 2.0:
            return (
                False,
                f"park [{self._target.name}] did not converge: worst joint error "
                f"{error:.4f} rad > {self.tolerance}",
            )
        return None

    def cancel(self) -> None:
        """Freeze the arms where they are by re-commanding the measured pose."""
        if self.enable and self.positions is not None and self._target is not None:
            self._publish_arms(list(self.positions))
        self._target = None
        self._start_time = None

    # -- publishing ----------------------------------------------------------

    def _publish_arms(self, canonical: Sequence[float]) -> None:
        left = Float64MultiArray()
        left.data = [float(v) for v in canonical[ARM_SLICE["left"]]]
        right = Float64MultiArray()
        right.data = [float(v) for v in canonical[ARM_SLICE["right"]]]
        self._left_pub.publish(left)
        self._right_pub.publish(right)

    def _publish_grippers(self, pose: ParkPose) -> None:
        if not self.command_grippers:
            return
        for side, position in (
            ("left", pose.left_gripper),
            ("right", pose.right_gripper),
        ):
            client = self._grippers[side]
            if not client.server_is_ready():
                self.log.warning(
                    f"{side} gripper action server not ready -- park leaves it as it is"
                )
                continue
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(self.gripper_max_effort)
            client.send_goal_async(goal)

    def _pose_error(self, target: Sequence[float]) -> Optional[float]:
        if self.positions is None:
            return None
        return max(
            abs(self.positions[i] - target[i])
            for side in ("left", "right")
            for i in range(ARM_SLICE[side].start, ARM_SLICE[side].stop)
        )

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9


class ArmParkNode(Node):
    """Standalone park, for checking a profile without running a mission."""

    def __init__(self) -> None:
        super().__init__("arm_park")
        self.declare_parameter("enable_park", False)
        self.declare_parameter("profile", "travel")
        self.declare_parameter("park_poses_file", "")
        self.declare_parameter("max_joint_speed", 0.15)
        self.declare_parameter("gripper_max_effort", 10.0)
        self.declare_parameter("park_tolerance", 0.01)
        self.declare_parameter("command_grippers", True)
        self.declare_parameter("state_wait_s", 5.0)

        get = self.get_parameter
        self.profile = str(get("profile").value)
        self.parker = ArmParker(
            self,
            poses=load_park_poses(str(get("park_poses_file").value)),
            enable=bool(get("enable_park").value),
            max_joint_speed=float(get("max_joint_speed").value),
            gripper_max_effort=float(get("gripper_max_effort").value),
            tolerance=float(get("park_tolerance").value),
            command_grippers=bool(get("command_grippers").value),
        )
        self._deadline = (
            self.get_clock().now().nanoseconds / 1e9 + float(get("state_wait_s").value)
        )
        self._started = False
        self.result: Optional[Tuple[bool, str]] = None
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._started:
            if not self.parker.has_state and now < self._deadline:
                return
            error = self.parker.start(self.profile)
            if error is not None:
                self.result = (False, error)
                return
            self._started = True
            return
        outcome = self.parker.poll()
        if outcome is not None:
            self.result = outcome


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ArmParkNode()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.parker.cancel()
    finally:
        ok, reason = node.result or (False, "interrupted")
        node.get_logger().info(f"park {'ok' if ok else 'FAILED'}: {reason}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
