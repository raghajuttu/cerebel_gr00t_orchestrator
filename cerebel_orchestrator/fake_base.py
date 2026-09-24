"""A chassis that exists only in odometry -- for testing Nav2 on a desk.

It subscribes to the twist the base adapter emits, integrates it, and publishes
the resulting ``/odom`` and ``odom -> base_link`` transform. Nav2 then plans,
drives and reports arrival against a robot that cannot hurt anybody, which is
enough to check the station poses, the goal tolerances, the interlock heartbeat
and the whole mission sequence before any of it reaches hardware.

It also publishes ``/all_wheel_rpm``, which is what the real chassis offers and
all it offers -- four wheel speeds, no pose. That is what ``move_base`` steps
integrate, so a mission that drives by wheel arc can be rehearsed here too. The
conversion is the inverse of the real thing: speed over the ground divided by
the wheel perimeter, published on all four wheels with the sign pattern the
vendor driver uses.

It is a perfect actuator: it goes exactly where it is told at exactly the
commanded velocity. Real wheels slip, and a mission that only works here is not
validated -- it is rehearsed.

    ros2 run cerebel_orchestrator fake_base --ros-args -p holonomic:=true
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .base_move import WHEEL_PERIMETER_M

# move_base.py's params.yaml value. Only used to turn a commanded spin into
# a plausible wheel speed; the mover never sees it.
CHASSIS_RADIUS_M = 0.0875
from tf2_ros import TransformBroadcaster


class FakeBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_base")
        self.declare_parameter("cmd_topic", "/base/cmd_vel")
        self.declare_parameter("cmd_type", "Twist")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("wheel_rpm_topic", "/all_wheel_rpm")
        self.declare_parameter("holonomic", False)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)

        get = self.get_parameter
        self.odom_frame = str(get("odom_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.holonomic = bool(get("holonomic").value)

        self.x = float(get("start_x").value)
        self.y = float(get("start_y").value)
        self.yaw = float(get("start_yaw").value)
        self.vx = self.vy = self.wz = 0.0

        message_type = Twist if str(get("cmd_type").value) == "Twist" else TwistStamped
        self.create_subscription(message_type, str(get("cmd_topic").value), self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, str(get("odom_topic").value), 10)
        self._rpm_pub = self.create_publisher(
            Float64MultiArray, str(get("wheel_rpm_topic").value), 10
        )
        self._tf = TransformBroadcaster(self)

        self.period = 1.0 / max(float(get("rate_hz").value), 1.0)
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f"fake_base: listening on {get('cmd_topic').value} "
            f"({get('cmd_type').value}), {'holonomic' if self.holonomic else 'differential'}, "
            f"starting at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})"
        )

    def _on_cmd(self, msg) -> None:
        twist = msg if isinstance(msg, Twist) else msg.twist
        self.vx = float(twist.linear.x)
        self.vy = float(twist.linear.y) if self.holonomic else 0.0
        self.wz = float(twist.angular.z)

    def _publish_wheel_rpm(self) -> None:
        """What the real chassis publishes: four speeds, and nothing about pose.

        The magnitude is the one that matters -- ``BaseMover`` takes the mean of
        the absolute values, because the real driver's signs are inconsistent
        between translation and rotation. The sign pattern here mirrors the
        vendor's [+1, -1, -1, -1] so that anything reading the raw array sees
        the shape it will see on hardware.
        """
        speed = math.hypot(self.vx, self.vy) + abs(self.wz) * CHASSIS_RADIUS_M
        rpm = speed / WHEEL_PERIMETER_M * 60.0
        message = Float64MultiArray()
        message.data = [rpm, -rpm, -rpm, -rpm]
        self._rpm_pub.publish(message)

    def _tick(self) -> None:
        dt = self.period
        # Body-frame velocities rotated into the odom frame.
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))

        stamp = self.get_clock().now().to_msg()
        rotation = Quaternion()
        rotation.z = math.sin(self.yaw / 2.0)
        rotation.w = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = rotation
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self._publish_wheel_rpm()
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = rotation
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeBaseNode()
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
