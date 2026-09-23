"""Between Nav2 and the vendor chassis: the interlock, and the shims.

Nav2 assumes one interface: it publishes ``geometry_msgs/Twist`` on ``/cmd_vel``
and expects ``nav_msgs/Odometry`` plus an ``odom -> base_link`` transform back. A
vendor chassis stack usually gets some of that right and some of it not. This
node sits in between and does four jobs:

**1. The interlock.** Wheel commands reach the base only while the orchestrator
is publishing ``base_enable = true``. The gate is fail-closed on a timeout, so it
is a heartbeat, not a latch: if the orchestrator crashes, is paused, or is busy
running a policy, the base stops within ``enable_timeout_s``. This is the one
mechanism that keeps the base from driving while the arms are working, and it
holds even if the mission logic above it is wrong.

**2. Zero-holding.** Most chassis drivers latch the last twist they were given
and keep driving. So while the gate is shut this node publishes zeros at
``zero_rate_hz`` rather than simply staying quiet, and it does the same when
Nav2 goes silent for ``cmd_timeout_s`` mid-goal.

**3. Clamping.** Every forwarded twist is clipped to ``max_linear`` /
``max_angular``, and with ``allow_lateral:=false`` the lateral component is
forced to zero. A differential base handed a ``linear.y`` from a holonomic
parameter file will either ignore it or do something surprising; this makes the
kinematic assumption explicit at the last point before the hardware.

**4. Shims.** Optional ``TwistStamped`` output for drivers that want it, topic
renaming in both directions, and an ``odom -> base_link`` broadcast derived from
the odometry message for drivers that publish odometry without TF. Nav2 will not
plan at all without that transform.

Everything is a parameter because the real answers are unknown until
``probe_robot`` has been run on the robot -- see docs/HARDWARE_PROBE.md.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class BaseAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("base_adapter")

        # --- topics in and out ---------------------------------------------
        self.declare_parameter("nav_cmd_topic", "/cmd_vel")
        self.declare_parameter("base_cmd_topic", "/base/cmd_vel")
        self.declare_parameter("base_cmd_type", "Twist")  # Twist | TwistStamped
        self.declare_parameter("vendor_odom_topic", "/odom")
        self.declare_parameter("odom_topic", "")  # "" -> do not republish
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_odom_tf", False)

        # --- the interlock --------------------------------------------------
        self.declare_parameter("enable_topic", "/orchestrator/base_enable")
        self.declare_parameter("enable_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("zero_rate_hz", 20.0)

        # --- limits ---------------------------------------------------------
        self.declare_parameter("max_linear", 0.25)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("allow_lateral", False)
        self.declare_parameter("require_enable", True)

        get = self.get_parameter
        self.base_cmd_type = str(get("base_cmd_type").value)
        if self.base_cmd_type not in ("Twist", "TwistStamped"):
            raise ValueError(f"base_cmd_type must be Twist or TwistStamped, got {self.base_cmd_type}")
        self.odom_frame = str(get("odom_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.enable_timeout_s = float(get("enable_timeout_s").value)
        self.cmd_timeout_s = float(get("cmd_timeout_s").value)
        self.max_linear = float(get("max_linear").value)
        self.max_angular = float(get("max_angular").value)
        self.allow_lateral = bool(get("allow_lateral").value)
        self.require_enable = bool(get("require_enable").value)

        message_type = Twist if self.base_cmd_type == "Twist" else TwistStamped
        self._cmd_pub = self.create_publisher(message_type, str(get("base_cmd_topic").value), 10)
        self.create_subscription(
            Twist, str(get("nav_cmd_topic").value), self._on_nav_cmd, 10
        )
        self.create_subscription(
            Bool, str(get("enable_topic").value), self._on_enable, 10
        )

        odom_out = str(get("odom_topic").value)
        self._odom_pub = (
            self.create_publisher(Odometry, odom_out, 10) if odom_out else None
        )
        self._tf = TransformBroadcaster(self) if bool(get("publish_odom_tf").value) else None
        if self._odom_pub is not None or self._tf is not None:
            self.create_subscription(
                Odometry,
                str(get("vendor_odom_topic").value),
                self._on_odom,
                qos_profile_sensor_data,
            )

        self._enabled = False
        self._enable_stamp: Optional[float] = None
        self._cmd_stamp: Optional[float] = None
        self._holding_zero = False
        self._odom_seen = False
        self.forwarded = 0
        self.blocked = 0

        self.create_timer(1.0 / max(float(get("zero_rate_hz").value), 1.0), self._tick)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"base_adapter: {get('nav_cmd_topic').value} -> {get('base_cmd_topic').value} "
            f"as {self.base_cmd_type}, gate {get('enable_topic').value} "
            f"(required: {self.require_enable}), limits {self.max_linear} m/s / "
            f"{self.max_angular} rad/s, lateral {'allowed' if self.allow_lateral else 'blocked'}"
        )
        if not self.require_enable:
            self.get_logger().warning(
                "require_enable is false: the base will accept Nav2 commands even "
                "while the arms are running. Bring-up only."
            )

    # -- the gate ------------------------------------------------------------

    def _on_enable(self, msg: Bool) -> None:
        was = self.gate_open
        self._enabled = bool(msg.data)
        self._enable_stamp = self._now()
        if self.gate_open != was:
            self.get_logger().info(f"base gate {'OPEN' if self.gate_open else 'SHUT'}")

    @property
    def gate_open(self) -> bool:
        if not self.require_enable:
            return True
        if not self._enabled or self._enable_stamp is None:
            return False
        return (self._now() - self._enable_stamp) <= self.enable_timeout_s

    # -- commands ------------------------------------------------------------

    def _on_nav_cmd(self, msg: Twist) -> None:
        if not self.gate_open:
            self.blocked += 1
            return
        out = Twist()
        out.linear.x = _clamp(msg.linear.x, self.max_linear)
        out.linear.y = _clamp(msg.linear.y, self.max_linear) if self.allow_lateral else 0.0
        out.angular.z = _clamp(msg.angular.z, self.max_angular)
        if not self.allow_lateral and abs(msg.linear.y) > 1e-3:
            self.get_logger().warning(
                f"dropped lateral command {msg.linear.y:.3f} m/s -- allow_lateral is false",
                throttle_duration_sec=5.0,
            )
        self._publish(out)
        self._cmd_stamp = self._now()
        self._holding_zero = False
        self.forwarded += 1

    def _tick(self) -> None:
        """Hold the base at zero whenever nothing valid is arriving."""
        gate = self.gate_open
        stale = self._cmd_stamp is None or (self._now() - self._cmd_stamp) > self.cmd_timeout_s
        if gate and not stale:
            return
        if gate and stale and not self._holding_zero:
            self.get_logger().warning(
                f"no /cmd_vel for {self.cmd_timeout_s}s while the gate is open -- holding zero"
            )
        self._publish(Twist())
        self._holding_zero = True

    def _publish(self, twist: Twist) -> None:
        if self.base_cmd_type == "Twist":
            self._cmd_pub.publish(twist)
            return
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.base_frame
        stamped.twist = twist
        self._cmd_pub.publish(stamped)

    # -- odometry ------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        if not self._odom_seen:
            self._odom_seen = True
            self.get_logger().info(
                f"odometry seen: frame {msg.header.frame_id!r} -> "
                f"child {msg.child_frame_id!r}"
            )
        if self._odom_pub is not None:
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.odom_frame
            out.child_frame_id = self.base_frame
            out.pose = msg.pose
            out.twist = msg.twist
            self._odom_pub.publish(out)
        if self._tf is not None:
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self._tf.sendTransform(transform)

    # -- housekeeping --------------------------------------------------------

    def _report(self) -> None:
        if self.blocked:
            self.get_logger().info(
                f"gate: {self.forwarded} forwarded, {self.blocked} blocked since start"
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # One last zero on the way out, so the base does not coast on a latched
        # command if this node is the thing that died.
        try:
            node._publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
