"""Arms that exist only in ``/joint_states`` -- for testing phase termination.

Publishes the 16 canonical joints so the orchestrator's grasp and settle checks
have something to read while no real robot is present, and so ``park_arms`` has a
pose to ramp from. Two ways to drive the gripper:

* **scripted** (default): open for ``grasp_after_s``, then closed. A dry-run
  mission with ``until: {grasp: closed, side: left}`` then completes its pick
  phase on its own, and the whole mission sequence runs start to finish on a
  laptop.
* **manual**: publish to ``/fake_arm/grasp`` (``std_msgs/Bool``) to open and close
  it by hand while watching the orchestrator's decisions.

The arm joints follow whatever is commanded on the controller topics, so a park
ramp converges here exactly as it would on a robot with a perfect servo -- which
is to say, better than the real one. This mock proves the plumbing, never the
motion.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from .joints import ARM_SLICE, CANONICAL_JOINT_ORDER, GRIPPER_INDEX


class FakeArmNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_arm")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("open_m", 0.040)
        self.declare_parameter("closed_m", 0.002)
        # <= 0 disables the script; the grasp then only follows /fake_arm/grasp.
        self.declare_parameter("grasp_after_s", 8.0)
        self.declare_parameter("release_after_s", -1.0)
        self.declare_parameter("follow_commands", True)
        self.declare_parameter("servo_rate", 0.5)  # fraction of the error per tick

        get = self.get_parameter
        self.open_m = float(get("open_m").value)
        self.closed_m = float(get("closed_m").value)
        self.grasp_after_s = float(get("grasp_after_s").value)
        self.release_after_s = float(get("release_after_s").value)
        self.servo_rate = min(max(float(get("servo_rate").value), 0.01), 1.0)

        self.positions: List[float] = [0.0] * 16
        self.positions[GRIPPER_INDEX["left"]] = self.open_m
        self.positions[GRIPPER_INDEX["right"]] = self.open_m
        self.targets: List[float] = list(self.positions)
        self._grasp_override: Optional[bool] = None

        if bool(get("follow_commands").value):
            self.create_subscription(
                Float64MultiArray,
                "/left_forward_position_controller/commands",
                self._make_arm_cb("left"),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                "/right_forward_position_controller/commands",
                self._make_arm_cb("right"),
                10,
            )
        self.create_subscription(Bool, "/fake_arm/grasp", self._on_grasp, 10)
        self._pub = self.create_publisher(JointState, "/joint_states", 10)

        self.started = self._now()
        self.create_timer(1.0 / max(float(get("rate_hz").value), 1.0), self._tick)
        self.get_logger().info(
            "fake_arm: publishing 16 canonical joints on /joint_states; "
            f"grasp closes at {self.grasp_after_s:.0f}s"
            + (f", releases at {self.release_after_s:.0f}s" if self.release_after_s > 0 else "")
        )

    def _make_arm_cb(self, side: str):
        def callback(msg: Float64MultiArray) -> None:
            span = ARM_SLICE[side]
            if len(msg.data) != span.stop - span.start:
                self.get_logger().warning(
                    f"{side} command has {len(msg.data)} values, expected "
                    f"{span.stop - span.start}"
                )
                return
            for offset, value in enumerate(msg.data):
                self.targets[span.start + offset] = float(value)

        return callback

    def _on_grasp(self, msg: Bool) -> None:
        self._grasp_override = bool(msg.data)
        self.get_logger().info(f"grasp override: {'closed' if msg.data else 'open'}")

    def _tick(self) -> None:
        elapsed = self._now() - self.started

        if self._grasp_override is not None:
            closed = self._grasp_override
        elif self.grasp_after_s <= 0:
            closed = False
        elif self.release_after_s > 0 and elapsed >= self.release_after_s:
            closed = False
        else:
            closed = elapsed >= self.grasp_after_s
        finger = self.closed_m if closed else self.open_m
        for side in ("left", "right"):
            self.targets[GRIPPER_INDEX[side]] = finger

        for index, target in enumerate(self.targets):
            self.positions[index] += (target - self.positions[index]) * self.servo_rate

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        # Scrambled on purpose: the real broadcaster does not publish in canonical
        # order, and anything that reads this by index rather than by name should
        # break here rather than on the robot.
        order = list(range(16))
        order = order[8:] + order[:8]
        message.name = [CANONICAL_JOINT_ORDER[i] for i in order]
        message.position = [self.positions[i] for i in order]
        message.velocity = [0.0] * 16
        message.effort = [0.0] * 16
        self._pub.publish(message)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
