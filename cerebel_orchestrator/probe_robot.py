"""A read-only survey of what the robot actually exposes.

Run this on the robot computer, with the normal bring-up running, before
configuring anything:

    ros2 run cerebel_orchestrator probe_robot --ros-args -p duration_s:=10.0

It publishes nothing and calls nothing -- it only listens -- so it is safe to run
next to a live system. It answers the questions that decide the whole
configuration:

* Does the chassis already accept a velocity command, and on which topic and
  type?
* Is there odometry, and is there an ``odom -> base_link`` transform? Nav2 will
  not plan without that transform, whatever else is present.
* Is there a lidar? That is what decides whether localisation can ever be more
  than dead reckoning.
* Are the 16 canonical arm joints all present in ``/joint_states``?
* Are the gripper action servers and the Nav2 action server up?
* Any hint of the wheel kinematics in the controller and topic names?

It finishes by writing a params file whose values are what it found, so the
adapter is configured from evidence rather than from guesses. Read it before
using it: a guess that came from a topic name is still a guess, and the file says
which values are which.
"""

from __future__ import annotations

import sys
from collections import defaultdict
from typing import Dict, List, Set, Tuple

import rclpy
import yaml
from rclpy.action import get_action_names_and_types
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from .joints import CANONICAL_JOINT_ORDER

TWIST_TYPES = ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped")
SCAN_TYPES = ("sensor_msgs/msg/LaserScan", "sensor_msgs/msg/PointCloud2")
IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")

KINEMATIC_HINTS = {
    "mecanum": "mecanum (holonomic -- can strafe)",
    "omni": "omnidirectional (holonomic -- can strafe)",
    "holonomic": "holonomic",
    "diff": "differential / skid-steer (cannot strafe)",
    "skid": "differential / skid-steer (cannot strafe)",
    "ackermann": "steered / Ackermann (no rotation in place)",
    "tricycle": "tricycle (no rotation in place)",
}

STATIC_TF_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class ProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("probe_robot")
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("out_file", "base_adapter.probed.yaml")
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.out_file = str(self.get_parameter("out_file").value)

        self.tf_edges: Set[Tuple[str, str]] = set()
        self.static_tf_edges: Set[Tuple[str, str]] = set()
        self.joint_names: Set[str] = set()
        self.joint_message_count = 0
        self.odom_frames: Dict[str, Tuple[str, str]] = {}
        self.odom_counts: Dict[str, int] = defaultdict(int)
        self.scan_counts: Dict[str, int] = defaultdict(int)

        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, STATIC_TF_QOS)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, qos_profile_sensor_data
        )
        # Odometry and scan subscriptions are created once the graph is known,
        # so the probe follows whatever names this robot uses.
        self._dynamic_subs_made = False

    # -- collectors ----------------------------------------------------------

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_edges.add((transform.header.frame_id, transform.child_frame_id))

    def _on_tf_static(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.static_tf_edges.add((transform.header.frame_id, transform.child_frame_id))

    def _on_joints(self, msg: JointState) -> None:
        self.joint_message_count += 1
        self.joint_names.update(msg.name)

    def _make_odom_cb(self, topic: str):
        def callback(msg: Odometry) -> None:
            self.odom_counts[topic] += 1
            self.odom_frames[topic] = (msg.header.frame_id, msg.child_frame_id)

        return callback

    def _make_scan_cb(self, topic: str):
        def callback(_msg) -> None:
            self.scan_counts[topic] += 1

        return callback

    def subscribe_to_found_topics(self, topics: List[Tuple[str, List[str]]]) -> None:
        if self._dynamic_subs_made:
            return
        self._dynamic_subs_made = True
        for name, types in topics:
            if "nav_msgs/msg/Odometry" in types:
                self.create_subscription(
                    Odometry, name, self._make_odom_cb(name), qos_profile_sensor_data
                )
            elif "sensor_msgs/msg/LaserScan" in types:
                self.create_subscription(
                    LaserScan, name, self._make_scan_cb(name), qos_profile_sensor_data
                )


def _section(title: str) -> None:
    print(f"\n== {title} " + "=" * max(0, 60 - len(title)))


def _endpoint_nodes(node: Node, topic: str, subscribers: bool) -> List[str]:
    try:
        infos = (
            node.get_subscriptions_info_by_topic(topic)
            if subscribers
            else node.get_publishers_info_by_topic(topic)
        )
    except Exception:  # pragma: no cover -- rmw dependent
        return []
    return sorted({info.node_name for info in infos})


def report(node: ProbeNode) -> Dict[str, object]:
    topics = sorted(node.get_topic_names_and_types())
    services = sorted(node.get_service_names_and_types())
    actions = sorted(get_action_names_and_types(node))
    node_names = sorted(f"{ns.rstrip('/')}/{name}".replace("//", "/") for name, ns in node.get_node_names_and_namespaces())

    findings: Dict[str, object] = {}

    _section("nodes")
    for name in node_names:
        print(f"  {name}")

    _section("velocity command topics (candidates for the chassis input)")
    twist_topics = [(name, types) for name, types in topics if any(t in TWIST_TYPES for t in types)]
    if not twist_topics:
        print("  none. The chassis has no ROS velocity interface -- a driver has to be written,")
        print("  or the vendor stack was not running when this probe ran.")
    for name, types in twist_topics:
        subs = _endpoint_nodes(node, name, subscribers=True)
        pubs = _endpoint_nodes(node, name, subscribers=False)
        verdict = "DRIVES THE BASE" if subs else "nobody is listening"
        print(f"  {name}  [{', '.join(types)}]")
        print(f"      subscribers: {subs or 'none'}  <- {verdict}")
        print(f"      publishers : {pubs or 'none'}")
    # The best candidate is a twist topic with a subscriber that is not Nav2.
    candidates = [
        (name, types)
        for name, types in twist_topics
        if [n for n in _endpoint_nodes(node, name, subscribers=True) if "nav" not in n.lower()]
    ]
    findings["base_cmd_topic"] = candidates[0][0] if candidates else "/base/cmd_vel"
    findings["base_cmd_type"] = (
        "TwistStamped"
        if candidates and "geometry_msgs/msg/TwistStamped" in candidates[0][1]
        else "Twist"
    )

    _section("odometry")
    odom_topics = [name for name, types in topics if "nav_msgs/msg/Odometry" in types]
    if not odom_topics:
        print("  none. Nav2 cannot localise at all without odometry.")
    for name in odom_topics:
        count = node.odom_counts.get(name, 0)
        frames = node.odom_frames.get(name)
        frame_text = f"{frames[0]!r} -> {frames[1]!r}" if frames else "no message seen"
        print(f"  {name}: {count} messages, frames {frame_text}")
    findings["vendor_odom_topic"] = odom_topics[0] if odom_topics else "/odom"

    _section("TF")
    all_edges = node.tf_edges | node.static_tf_edges
    if not all_edges:
        print("  no transforms at all.")
    for parent, child in sorted(all_edges):
        kind = "static" if (parent, child) in node.static_tf_edges else "dynamic"
        print(f"  {parent} -> {child}  ({kind})")
    odom_children = {child for parent, child in all_edges if parent == "odom"}
    has_odom_tf = bool(odom_children)
    base_frame = "base_link"
    if odom_children:
        base_frame = sorted(odom_children)[0]
    elif node.odom_frames:
        base_frame = next(iter(node.odom_frames.values()))[1] or "base_link"
    findings["base_frame"] = base_frame
    findings["publish_odom_tf"] = not has_odom_tf
    print(
        f"  -> odom -> {base_frame}: "
        + ("present" if has_odom_tf else "MISSING; the adapter must broadcast it")
    )
    if any(parent == "map" for parent, _ in all_edges):
        print("  -> a map frame already exists; check nothing else is publishing map -> odom")

    _section("lidar / range sensors")
    scan_topics = [(name, types) for name, types in topics if any(t in SCAN_TYPES for t in types)]
    if not scan_topics:
        print("  none. Localisation can only be dead reckoning from wheel odometry;")
        print("  AMCL and obstacle avoidance are both off the table until a lidar exists.")
    for name, types in scan_topics:
        count = node.scan_counts.get(name, 0)
        print(f"  {name}  [{', '.join(types)}]  {count} messages")
    findings["has_lidar"] = bool(scan_topics)

    _section("arm joints")
    print(f"  /joint_states messages seen: {node.joint_message_count}")
    missing = [name for name in CANONICAL_JOINT_ORDER if name not in node.joint_names]
    extra = sorted(node.joint_names - set(CANONICAL_JOINT_ORDER))
    if node.joint_message_count == 0:
        print("  nothing published -- the arm bring-up is not running")
    elif missing:
        print(f"  MISSING {len(missing)} canonical joints: {missing}")
    else:
        print("  all 16 canonical joints present")
    if extra:
        print(f"  other joints present (wheels?): {extra}")
    findings["joint_states_ok"] = node.joint_message_count > 0 and not missing
    findings["extra_joints"] = extra

    _section("cameras")
    image_topics = [name for name, types in topics if any(t in IMAGE_TYPES for t in types)]
    for name in image_topics:
        print(f"  {name}")
    if not image_topics:
        print("  none -- the policy cannot run without its three RGB streams")

    _section("action servers")
    for name, types in actions:
        print(f"  {name}  [{', '.join(types)}]")
    action_names = [name for name, _ in actions]
    for wanted in (
        "navigate_to_pose",
        "/left_gripper_controller/gripper_cmd",
        "/right_gripper_controller/gripper_cmd",
    ):
        found = any(wanted.lstrip("/") in name.lstrip("/") for name in action_names)
        print(f"  -> {wanted}: {'present' if found else 'NOT FOUND'}")
    findings["nav_action_present"] = any("navigate_to_pose" in name for name in action_names)

    _section("kinematics hints (from names only -- confirm mechanically)")
    haystack = " ".join(
        [name for name, _ in topics] + [name for name, _ in services] + node_names
    ).lower()
    hits = sorted({label for key, label in KINEMATIC_HINTS.items() if key in haystack})
    for hit in hits:
        print(f"  {hit}")
    if not hits:
        print("  nothing in the names says. Look at the wheels: rollers on the tyres")
        print("  (mecanum/omni) means it can strafe; plain tyres mean it cannot.")
    findings["kinematics_hints"] = hits
    findings["allow_lateral"] = any("holonomic" in hit or "strafe" in hit for hit in hits)

    if any("controller_manager" in name for name, _ in services):
        print("\n  ros2_control is running; list the controllers for the real answer:")
        print("      ros2 control list_controllers")

    return findings


def write_params(findings: Dict[str, object], path: str) -> None:
    document = {
        "base_adapter": {
            "ros__parameters": {
                "nav_cmd_topic": "/cmd_vel",
                "base_cmd_topic": findings.get("base_cmd_topic"),
                "base_cmd_type": findings.get("base_cmd_type"),
                "vendor_odom_topic": findings.get("vendor_odom_topic"),
                "odom_topic": "",
                "odom_frame": "odom",
                "base_frame": findings.get("base_frame"),
                "publish_odom_tf": findings.get("publish_odom_tf"),
                "allow_lateral": findings.get("allow_lateral", False),
                "max_linear": 0.15,
                "max_angular": 0.4,
                "require_enable": True,
            }
        }
    }
    header = (
        "# Written by `ros2 run cerebel_orchestrator probe_robot`.\n"
        "#\n"
        "# Values taken from what was observed: base_cmd_topic, base_cmd_type,\n"
        "# vendor_odom_topic, base_frame, publish_odom_tf.\n"
        "# Values that are GUESSES and must be confirmed by hand:\n"
        "#   allow_lateral  -- inferred from names; look at the wheels instead\n"
        "#   max_linear / max_angular -- deliberately slow; raise only after a\n"
        "#                               gated test with the e-stop in reach\n"
        f"# probe findings: {findings}\n"
    )
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(header)
        yaml.safe_dump(document, handle, sort_keys=False)
    print(f"\nwrote {path}")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ProbeNode()
    try:
        print(f"listening for {node.duration_s:.0f}s ...")
        # First spin discovers the graph, then subscribe to what was found and
        # spin again to actually catch messages on those topics.
        deadline = node.get_clock().now().nanoseconds / 1e9 + node.duration_s * 0.3
        while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.subscribe_to_found_topics(node.get_topic_names_and_types())
        deadline = node.get_clock().now().nanoseconds / 1e9 + node.duration_s * 0.7
        while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        findings = report(node)
        write_params(findings, node.out_file)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
