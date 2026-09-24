"""Nav2 ``NavigateToPose``, wrapped so the orchestrator never blocks.

Everything here is poll-shaped: ``send`` starts a goal and returns immediately,
``poll`` returns None while the goal is in flight and ``(ok, reason)`` once it is
not. That is what lets one timer callback drive the whole mission on a
single-threaded executor -- nothing in the orchestrator ever waits on a future,
so the e-stop subscription is always live.

Nav2 is used **without SLAM and without AMCL**: the global costmap is a rolling
window and ``map`` is pinned to ``odom`` by a static transform, so goals are
odometry-relative. See docs/NAVIGATION.md for why, and for what that costs.

**Nav2 is an optional dependency.** A mission made only of ``run_policy``,
``park_arms`` and ``move_base`` steps never navigates, and on a robot where
``nav2_msgs`` is not installed it must still run. So the import is guarded and
this module stays importable either way: without Nav2 the client exists but
refuses to send, and the orchestrator turns that into a clear startup error --
but only for a mission that actually has a ``navigate`` step.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from nav2_msgs.action import NavigateToPose

    NAV2_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on what is installed
    NavigateToPose = None
    NAV2_AVAILABLE = False

NAV2_MISSING = (
    "nav2_msgs is not installed on this machine, so `navigate` steps cannot "
    "run. Install the Nav2 stack, or use `move_base` steps instead -- see "
    "docs/NAVIGATION.md, which explains why this chassis cannot use Nav2 yet."
)

from .mission import Station


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def station_to_pose(station: Station, frame_id: str, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = station.x
    pose.pose.position.y = station.y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(station.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class NavClient:
    def __init__(self, node: Node, action_name: str = "navigate_to_pose") -> None:
        self.node = node
        self.action_name = action_name
        self.available = NAV2_AVAILABLE
        self._client = (
            ActionClient(node, NavigateToPose, action_name) if NAV2_AVAILABLE else None
        )
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._cancel_future = None
        self._deadline: Optional[float] = None
        self._started: Optional[float] = None
        self.distance_remaining: Optional[float] = None

    # -- server availability -------------------------------------------------

    def server_ready(self, timeout_s: float = 0.0) -> bool:
        if self._client is None:
            return False
        return self._client.wait_for_server(timeout_sec=timeout_s)

    # -- one goal ------------------------------------------------------------

    def send(self, station: Station, frame_id: str, timeout_s: float) -> None:
        if self._client is None:
            raise RuntimeError(NAV2_MISSING)
        if self.busy:
            raise RuntimeError("NavClient.send() while a goal is still in flight")
        goal = NavigateToPose.Goal()
        goal.pose = station_to_pose(station, frame_id, self.node.get_clock().now().to_msg())
        self.distance_remaining = None
        self._started = self._now()
        self._deadline = self._started + timeout_s
        self._goal_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)

    @property
    def busy(self) -> bool:
        return any(
            f is not None for f in (self._goal_future, self._result_future, self._cancel_future)
        )

    def poll(self) -> Optional[Tuple[bool, str]]:
        """None while navigating; (ok, reason) when the goal has resolved."""
        if self._cancel_future is not None:
            if self._cancel_future.done():
                self._reset()
                return (False, "goal cancelled")
            return None

        if self._goal_future is not None:
            if not self._goal_future.done():
                return self._check_deadline()
            handle = self._goal_future.result()
            self._goal_future = None
            if handle is None or not handle.accepted:
                self._reset()
                return (False, "Nav2 rejected the goal")
            self._goal_handle = handle
            self._result_future = handle.get_result_async()
            return self._check_deadline()

        if self._result_future is not None:
            if not self._result_future.done():
                return self._check_deadline()
            wrapped = self._result_future.result()
            elapsed = self._elapsed()
            self._reset()
            status = getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN)
            if status == GoalStatus.STATUS_SUCCEEDED:
                return (True, f"arrived in {elapsed:.1f}s")
            return (False, f"Nav2 finished with status {_status_name(status)} after {elapsed:.1f}s")

        return None

    def cancel(self, reason: str = "cancelled") -> None:
        """Ask Nav2 to stop. ``poll`` reports the cancellation once it lands."""
        if self._goal_handle is not None and self._cancel_future is None:
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._goal_future = None
            self._result_future = None
            return
        if self.busy:
            # The goal was never accepted, so there is nothing to cancel.
            self._reset()

    # -- internals -----------------------------------------------------------

    def _check_deadline(self) -> Optional[Tuple[bool, str]]:
        if self._deadline is not None and self._now() > self._deadline:
            remaining = (
                f", {self.distance_remaining:.2f} m short"
                if self.distance_remaining is not None
                else ""
            )
            self.cancel()
            self._reset()
            return (False, f"navigation timed out after {self._elapsed():.1f}s{remaining}")
        return None

    def _on_feedback(self, message) -> None:
        self.distance_remaining = float(message.feedback.distance_remaining)

    def _elapsed(self) -> float:
        return 0.0 if self._started is None else self._now() - self._started

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _reset(self) -> None:
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._cancel_future = None
        self._deadline = None


_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def _status_name(status: int) -> str:
    return _STATUS_NAMES.get(status, str(status))
