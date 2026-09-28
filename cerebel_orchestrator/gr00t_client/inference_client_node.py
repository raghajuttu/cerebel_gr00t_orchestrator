#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
adibot_gr00t_client.inference_client_node
=========================================
ROS2 (Humble / rclpy / Python 3.10) inference client bridging a bimanual
OpenArm robot to a remote NVIDIA GR00T policy server over ZMQ.

Formerly `adibot_deployment_log`. On top of the v0.1.0 baseline this build
adds per-run CSV logging (adibot_gr00t_client/data_logger.py), dataset-derived
soft limits, prefetch (next chunk requested on a worker thread while the
current one executes), and optional RTC seam stitching with the seed aligned
to the step the arm is actually on (adibot_gr00t_client/rtc.py) and verified
against every returned chunk. It installs as a SEPARATE package so the
validated `adibot_deployment` stays untouched:

    ros2 run adibot_deployment     inference_client       # validated v1
    ros2 run adibot_gr00t_client   inference_client       # this build

DEPLOYMENT MODE: BIMANUAL (16-D). Left arm does the pick-place task; right arm
is present in the model output. Each side is independently gated by an enable
flag (left-arm-first is the recommended safe bring-up).

Confirmed live topics (SER9):
  head        -> /head_cam/camera/color/image_raw
  wrist_left  -> /l_cam/camera/color/image_raw       (recorder maps l -> wrist_left)
  wrist_right -> /r_cam/camera/color/image_raw       (recorder maps r -> wrist_right)
  left arm    -> /left_forward_position_controller/commands
  right arm   -> /right_forward_position_controller/commands
  left grip   -> ACTION /left_gripper_controller/gripper_cmd  (GripperCommand,
  right grip  -> ACTION /right_gripper_controller/gripper_cmd  raw units, no scaling)
  state       -> /joint_states (scrambled; reordered by name into 16-D)

Transport: minimal vendored ZMQ REQ client (msgpack + msgpack_numpy).

Three correctness-critical sections flagged with `# >>> CRITICAL`:
  1. Joint reordering (subscribe side)     -> _joint_states_cb()
  2. Action parsing (server response side) -> _normalize_action()
  3. Gripper scaling (publish side)        -> _publish_gripper()

SAFETY -- per-joint soft limits (a GUARD, not a correction):
  The commanded action is checked against per-joint [lower, upper] bounds; if
  any enabled arm joint is out of range that whole side is SKIPPED (held), never
  clamped-and-sent. Limits come from `limits_file` (a YAML produced by
  scripts/extract_limits.py from the training dataset, in the arm's real encoder
  frame); joints not listed fall back to the global joint_min/joint_max. Because
  the bounds sit just outside the trained action range, a well-behaved policy
  never touches them -- only a policy diverging from its training gets caught. A
  sustained high skip rate raises an OUT-OF-DISTRIBUTION warning.
"""
import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import cv2
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import zmq
import msgpack
import msgpack_numpy as msgpack_numpy  # noqa: N812

from cerebel_orchestrator.gr00t_client.data_logger import InferenceLogger
from cerebel_orchestrator.gr00t_client.rtc import (build_seed_action, freeze_error_mrad,
                                     plan_seed, rtc_options, seed_rows)

# =============================================================================
# Canonical joint ordering (16-D) - AUTHORITATIVE
# =============================================================================
CANONICAL_JOINT_ORDER = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7", "openarm_left_finger_joint1",
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7", "openarm_right_finger_joint1",
]
# Slices within the 16-D canonical vector.
LEFT_ARM_SLICE = slice(0, 7)      # left joints 1..7
LEFT_GRIPPER_IDX = 7
RIGHT_ARM_SLICE = slice(8, 15)    # right joints 1..7
RIGHT_GRIPPER_IDX = 15

# Stamped into every run's sidecar so a log can be traced to the code that
# produced it. Keep in step with setup.py.
PACKAGE_VERSION = "1.0.0"

# =============================================================================
# Vendored minimal ZMQ client (REQ socket, msgpack + msgpack_numpy)
# =============================================================================
class GR00TZMQClient:
    """Tiny REQ-socket client speaking the GR00T inference protocol."""

    def __init__(self, host: str, port: int, timeout_ms: int = 15000):
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._ctx: Optional[zmq.Context] = None
        self._sock: Optional[zmq.Socket] = None
        self._connect()

    def _connect(self) -> None:
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None
        if self._ctx is None:
            self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(f"tcp://{self.host}:{self.port}")

    def reconnect(self) -> None:
        """Rebuild the socket. A REQ socket that times out mid-request is left
        in a broken (out-of-sync) state and MUST be recreated before reuse."""
        self._connect()

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None

    @staticmethod
    def _pack(payload) -> bytes:
        return msgpack.packb(payload, default=msgpack_numpy.encode,
                             use_bin_type=True)

    @staticmethod
    def _unpack(raw: bytes):
        return msgpack.unpackb(raw, object_hook=msgpack_numpy.decode, raw=False)

    def _request(self, payload):
        self._sock.send(self._pack(payload))
        raw = self._sock.recv()
        return self._unpack(raw)

    def ping(self) -> Tuple[bool, object]:
        try:
            resp = self._request({"endpoint": "ping"})
            return True, resp
        except zmq.error.Again:
            self.reconnect()
            return False, None

    def get_action(self, observation, options=None):
        resp = self._request(
            {"endpoint": "get_action",
             "data": {"observation": observation, "options": options}}
        )
        if isinstance(resp, (list, tuple)) and len(resp) == 2:
            action, info = resp[0], resp[1]
        else:
            action, info = resp, {}
        return action, info


# =============================================================================
# One inference request, as handed to the worker and echoed back with its
# response, so the receiver knows exactly what that request carried.
# =============================================================================
@dataclass
class _Request:
    seq: int                    # inference_seq of this request
    obs: Optional[dict]         # observation sent (the worker drops it once sent)
    options: Optional[dict]     # RTC options, or None
    req_tick: int               # executed-step count when the observation was taken
    t_req: float                # wall time of that observation
    obs_state: np.ndarray       # the 16-D joint state it carried (copy)
    rtc: Optional[dict]         # seed facts (see _attach_rtc_seed) or None


# =============================================================================
# The ROS2 node
# =============================================================================
class InferenceClientNode(Node):
    def __init__(self):
        # Distinct ROS node name so this cannot collide in the graph with the
        # plain adibot_deployment package's "inference_client" node.
        super().__init__("gr00t_client")
        # ---- Parameters -----------------------------------------------------
        self.declare_parameter("server_host", "192.168.8.153")
        self.declare_parameter("server_port", 5555)
        # 30 Hz: the robot cannot keep up at 50 (confirmed on hardware).
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("execution_horizon", 16)
        # ---- Prefetch -------------------------------------------------------
        # prefetch_enable: request the NEXT chunk on a worker thread while the
        # current one is still executing, instead of blocking the control loop
        # at every chunk boundary. False = original blocking behaviour.
        self.declare_parameter("prefetch_enable", True)
        # Fire the next request when this many steps remain in the buffer.
        # Must cover the WORST-CASE round trip in control ticks (p95 latency /
        # tick period), not the median -- a lead that only covers the median
        # stalls on every slow reply.
        self.declare_parameter("prefetch_lead", 12)
        # ---- RTC (real-time chunking) ---------------------------------------
        # rtc_enable: send the current action chunk back with each request so
        # the server grows the new chunk out of it (GR00T's RTC inpainting)
        # instead of fresh noise -- stitching consecutive chunks together at
        # the seam. The rows sent are aligned to the step the arm is on when
        # the request fires (see rtc.py), not just the chunk's tail. REQUIRES
        # a patched policy server (server/README.md); whether the server
        # honoured the seed is checked on every returned chunk (see
        # rtc_freeze_warn_mrad below).
        self.declare_parameter("rtc_enable", False)
        # Steps of the new chunk seeded from the current chunk, counted from
        # the step the arm is on. Cut down, per request, to the steps that
        # remain in the current chunk past that point.
        self.declare_parameter("rtc_overlap_steps", 12)
        # Steps of those held verbatim -- should cover the inference round
        # trip in ticks, since those steps play while the server thinks.
        # Never more than the (effective) overlap.
        self.declare_parameter("rtc_frozen_steps", 9)
        # Ramp rate for easing from frozen to freshly-denoised steps.
        self.declare_parameter("rtc_ramp_rate", 5.0)
        # Verification: the server holds the first rtc_frozen_steps of every
        # RTC chunk EXACTLY equal to the seed we sent (bf16 rounding aside, a
        # few mrad). If a returned chunk's frozen block strays from the seed
        # by more than this (x1000 joint units; mrad for the arm joints) the
        # server is ignoring the seed -- unpatched gr00t_policy.py, old code
        # on its import path, or not restarted -- and the node says so once.
        # 0 disables the check. The value is logged per chunk either way.
        self.declare_parameter("rtc_freeze_warn_mrad", 20.0)
        # NOTE: no gripper scaling. GripperCommand.position takes the finger
        # joint position in the SAME units the dataset/action uses (~0.0 closed
        # .. ~0.05 open, verified on hardware) -- the VLA output passes through
        # raw. The old divide-by-0.05 belonged to the exo bridge's 0..1
        # convention and would command ~20x past the finger's range here.
        # Per-side enables. Enable EXACTLY the actuators that moved during your
        # data collection (dataset-specific -- see docs/VALIDATION.md): actuating a
        # channel the policy never saw moving drives the observation
        # out-of-distribution and can collapse the whole rollout.
        self.declare_parameter("enable_left_arm", True)
        self.declare_parameter("enable_left_gripper", True)
        self.declare_parameter("enable_right_arm", True)
        self.declare_parameter("enable_right_gripper", True)
        # Fallback global soft limits, used for any arm joint NOT covered by a
        # per-joint limits file (and for every joint when no file is given).
        self.declare_parameter("joint_min", -3.14)
        self.declare_parameter("joint_max", 3.14)
        # Master switch for the soft-limit guard. False = publish every
        # commanded value unchecked (rely on the hardware e-stop). Default True.
        self.declare_parameter("enable_limits", True)
        # Per-joint soft limits derived from the training dataset (see
        # scripts/extract_limits.py). "" -> use the global joint_min/max for all.
        self.declare_parameter("limits_file", "")
        # If the fraction of recently-skipped steps exceeds this, warn that the
        # policy looks out-of-distribution. 0 disables the warning.
        self.declare_parameter("ood_warn_fraction", 0.3)
        self.declare_parameter("zmq_timeout_ms", 15000)
        # Run logging: actual joint values vs VLA output, written to disk.
        self.declare_parameter("enable_logging", True)
        self.declare_parameter("log_dir", "~/adibot_logs")
        self.declare_parameter("log_run_name", "")   # "" -> run_<timestamp>
        # Which checkpoint the SERVER is serving. The client cannot know this
        # -- it only sees actions -- so it is recorded here for the run's
        # sidecar. Leave "" if the server reports its model path in the action
        # info dict, which is captured automatically.
        self.declare_parameter("checkpoint_label", "")
        # Free-text note stored in the sidecar: object placement, lighting,
        # anything that would otherwise only live in the filename.
        self.declare_parameter("run_notes", "")
        # Record every full action chunk (executed and unexecuted steps) to
        # <run>.chunks.npz. Cheap (~2.5 KB/chunk); enables true chunk-overlap
        # analysis offline. Disable only if disk space is somehow a concern.
        self.declare_parameter("log_chunks", True)
        # MUST match the dataset's annotation string exactly -- language
        # conditions the policy.
        self.declare_parameter(
            "task_description",
            "pick up the lipstick and place it into the box",
        )
        # Topics (confirmed on SER9).
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("head_image_topic",
                               "/head_cam/camera/color/image_raw")
        self.declare_parameter("wrist_left_image_topic",
                               "/l_cam/camera/color/image_raw")
        self.declare_parameter("wrist_right_image_topic",
                               "/r_cam/camera/color/image_raw")
        self.declare_parameter("left_arm_command_topic",
                               "/left_forward_position_controller/commands")
        self.declare_parameter("right_arm_command_topic",
                               "/right_forward_position_controller/commands")
        # Grippers are driven by ros2_control GripperActionController via a
        # GripperCommand ACTION (verified on hardware: publishing any topic
        # does nothing -- the old /left|right_arm/joint_command path was the
        # exoskeleton bridge's input, dead during inference, 0 subscribers).
        self.declare_parameter("left_gripper_action",
                               "/left_gripper_controller/gripper_cmd")
        self.declare_parameter("right_gripper_action",
                               "/right_gripper_controller/gripper_cmd")
        # Max squeezing force cap forwarded in every GripperCommand goal.
        # 10.0 verified on hardware: opens/closes briskly, gentle stall.
        self.declare_parameter("gripper_max_effort", 10.0)
        # A new goal is sent only when the commanded position moved at least
        # this much (joint units) since the last goal -- avoids spamming the
        # action server at the control rate.
        self.declare_parameter("gripper_min_delta", 0.002)

        gp = self.get_parameter
        self.server_host = gp("server_host").value
        self.server_port = int(gp("server_port").value)
        self.control_rate_hz = float(gp("control_rate_hz").value)
        self.execution_horizon = int(gp("execution_horizon").value)
        self.prefetch_enable = bool(gp("prefetch_enable").value)
        self.prefetch_lead = int(gp("prefetch_lead").value)
        self.rtc_enable = bool(gp("rtc_enable").value)
        self.rtc_overlap_steps = int(gp("rtc_overlap_steps").value)
        self.rtc_frozen_steps = int(gp("rtc_frozen_steps").value)
        self.rtc_ramp_rate = float(gp("rtc_ramp_rate").value)
        self.rtc_freeze_warn_mrad = float(gp("rtc_freeze_warn_mrad").value)
        self.enable_left_arm = bool(gp("enable_left_arm").value)
        self.enable_left_gripper = bool(gp("enable_left_gripper").value)
        self.enable_right_arm = bool(gp("enable_right_arm").value)
        self.enable_right_gripper = bool(gp("enable_right_gripper").value)
        self.joint_min = float(gp("joint_min").value)
        self.joint_max = float(gp("joint_max").value)
        self.limits_file = str(gp("limits_file").value)
        self.enable_limits = bool(gp("enable_limits").value)
        self.ood_warn_fraction = float(gp("ood_warn_fraction").value)
        self.zmq_timeout_ms = int(gp("zmq_timeout_ms").value)
        self.enable_logging = bool(gp("enable_logging").value)
        self.log_dir = str(gp("log_dir").value)
        self.log_run_name = str(gp("log_run_name").value) or None
        self.checkpoint_label = str(gp("checkpoint_label").value)
        self.run_notes = str(gp("run_notes").value)
        self.log_chunks = bool(gp("log_chunks").value)
        self.task_description = str(gp("task_description").value)
        self.joint_states_topic = gp("joint_states_topic").value
        self.head_image_topic = gp("head_image_topic").value
        self.wrist_left_image_topic = gp("wrist_left_image_topic").value
        self.wrist_right_image_topic = gp("wrist_right_image_topic").value
        self.left_arm_command_topic = gp("left_arm_command_topic").value
        self.right_arm_command_topic = gp("right_arm_command_topic").value
        self.left_gripper_action = gp("left_gripper_action").value
        self.right_gripper_action = gp("right_gripper_action").value
        self.gripper_max_effort = float(gp("gripper_max_effort").value)
        self.gripper_min_delta = float(gp("gripper_min_delta").value)

        # ---- Per-joint soft limits ------------------------------------------
        # Build 16-D lower/upper arrays aligned to CANONICAL_JOINT_ORDER. Start
        # from the global fallback, then override per joint from limits_file if
        # given. Only the 14 ARM joints are ever checked; the two gripper slots
        # stay at the fallback and are never enforced.
        self.limit_lower = np.full(len(CANONICAL_JOINT_ORDER), self.joint_min,
                                   dtype=np.float64)
        self.limit_upper = np.full(len(CANONICAL_JOINT_ORDER), self.joint_max,
                                   dtype=np.float64)
        self.limits_source = "global joint_min/joint_max"
        if self.limits_file:
            self._load_limits_file(self.limits_file)

        # ---- State ----------------------------------------------------------
        self.bridge = CvBridge()
        self.latest_state: Optional[np.ndarray] = None      # (16,) float32
        # Velocity/effort are logged alongside position when the driver
        # publishes them; they stay None (-> nan in the CSV) if it does not.
        self.latest_velocity: Optional[np.ndarray] = None   # (16,) float32
        self.latest_effort: Optional[np.ndarray] = None     # (16,) float32
        self.latest_head: Optional[np.ndarray] = None       # (480,640,3) uint8
        self.latest_wrist_left: Optional[np.ndarray] = None
        self.latest_wrist_right: Optional[np.ndarray] = None
        self._logged_action_structure = False
        self._missing_joint_warned = set()
        # Buffer entries are (horizon_idx, 16-vector):
        #   [L_arm0..6, L_grip, R_arm0..6, R_grip]
        self._action_buffer = []
        # Log bookkeeping: monotonic tick counter, and which server response
        # the steps currently in the buffer came from.
        self._tick = 0
        self._inference_seq = 0
        self._current_inference_seq = -1
        # Per-chunk facts, logged on that chunk's first executed step only
        # (same convention as latency_ms) and cleared after.
        self._pending_latency_ms: Optional[float] = None
        self._pending_chunk_len: Optional[int] = None
        self._pending_skip: Optional[int] = None
        self._pending_rtc: Optional[bool] = None
        # Written once into the sidecar, not per chunk. A dedicated flag,
        # because _pending_chunk_len is cleared on every logged tick and so
        # cannot serve as an "already recorded" sentinel.
        self._chunk_len_logged = False
        # Requests that never became executed steps. Their inference_seq
        # numbers are missing from the CSV (the counter advances per REQUEST,
        # while only accepted chunks reach a row), so without these counts a
        # gap in inference_seq is unattributable.
        self._failed_requests = 0    # timeout / ZMQ error, no response
        self._dropped_chunks = 0     # response arrived but would not parse
        self._stale_chunks = 0       # arrived so late every step had expired
        # ---- Prefetch state -------------------------------------------------
        # The ZMQ REQ socket is NOT thread-safe. After start() hands it to the
        # worker thread (Thread.start() is a full memory barrier), only the
        # worker touches self.client. The control loop talks to the worker
        # exclusively through these queues.
        self._req_q: queue.Queue = queue.Queue(maxsize=1)
        self._resp_q: queue.Queue = queue.Queue()
        self._worker: Optional[threading.Thread] = None
        self._inflight = False
        self._stall_ticks = 0
        # ---- RTC state ------------------------------------------------------
        # The current plan = the last ACCEPTED chunk, kept both exactly as the
        # server returned it (dict of (1, H, D) arrays; re-indexed and sent
        # back as the seed) and in canonical (H, 16) form (freeze check and
        # chunk store). A chunk dropped as stale never becomes the plan.
        self._last_raw_action = None
        self._last_steps: Optional[np.ndarray] = None
        self._last_raw_horizon = 0
        self._last_seq = -1
        # Index of the plan step the arm executes next. Every executed step
        # advances it; the seed sent with a request starts at it, because the
        # new chunk's step 0 stands for the same instant (see rtc.py).
        self._chunk_cursor = 0
        self._rtc_form_warned = False
        self._rtc_reduced_warned = False
        self._rtc_freeze_warned = False
        self._rtc_ack_noted = False
        # RTC outcome counters, stamped into the sidecar at shutdown.
        self._rtc_requests = 0           # requests that carried a seed
        self._rtc_reduced_overlap = 0    # ...with the overlap cut by the plan's end
        self._rtc_freeze_violations = 0  # chunks whose frozen block missed the seed
        self._rtc_frozen_overrun = 0     # chunks back after their frozen block played out
        self._short_chunk_warned = False
        # Out-of-distribution monitor: rolling window of per-step skip flags
        # (a step is "skipped" when any enabled side was out of limits).
        self._skip_window = deque(maxlen=100)
        self._last_ood_warn_tick = 0

        self.client = GR00TZMQClient(self.server_host, self.server_port,
                                     timeout_ms=self.zmq_timeout_ms)

        # ---- Run logger ------------------------------------------------------
        self.data_log: Optional[InferenceLogger] = None
        if self.enable_logging:
            try:
                self.data_log = InferenceLogger(
                    log_dir=self.log_dir,
                    joint_names=CANONICAL_JOINT_ORDER,
                    run_name=self.log_run_name,
                    ros_logger=self.get_logger(),
                )
                self.data_log.write_meta(self._run_params())
            except Exception as exc:  # noqa: BLE001
                # Logging must never take the robot down.
                self.get_logger().error(
                    f"Could not start data logger ({exc}); continuing WITHOUT "
                    f"logging.")
                self.data_log = None

        # ---- Subscribers ----------------------------------------------------
        # /joint_states QoS: BEST_EFFORT to match the joint_state_broadcaster's
        # live stream. A default RELIABLE subscriber against that broadcaster's
        # RELIABLE+TRANSIENT_LOCAL profile latched the initial (zero) sample and
        # never tracked real motion, so every logged state read 0.0. run_probe.py
        # and robot_state_publisher both use BEST_EFFORT and both see real state;
        # this matches them.
        joint_states_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(JointState, self.joint_states_topic,
                                 self._joint_states_cb, joint_states_qos)
        self.create_subscription(Image, self.head_image_topic,
                                 self._make_image_cb("head"),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.wrist_left_image_topic,
                                 self._make_image_cb("wrist_left"),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.wrist_right_image_topic,
                                 self._make_image_cb("wrist_right"),
                                 qos_profile_sensor_data)

        # ---- Publishers / action clients ------------------------------------
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray, self.left_arm_command_topic, 10)
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray, self.right_arm_command_topic, 10)
        # Grippers: GripperCommand action clients (see param comment above).
        self.left_gripper_client = ActionClient(
            self, GripperCommand, self.left_gripper_action)
        self.right_gripper_client = ActionClient(
            self, GripperCommand, self.right_gripper_action)
        # Last goal position sent per side, for the min-delta filter.
        self._last_gripper_sent = {"left": None, "right": None}
        self._gripper_server_warned = set()

        self._print_startup_banner()
        self._timer = None  # created after a successful ping (see start())

    # ------------------------------------------------------------------ setup
    def _load_limits_file(self, path: str) -> None:
        """Override per-joint limits from a YAML produced by extract_limits.py.

        Format:
            joints:
              openarm_left_joint1: {lower: -2.01, upper: 0.23}
              ...
        Unknown joint names are ignored; any joint absent from the file keeps
        the global fallback. On any error we log and keep the fallback -- a bad
        limits file must never stop the robot from starting.
        """
        import os
        log = self.get_logger()
        try:
            import yaml
            full = os.path.expanduser(path)
            with open(full, "r") as f:
                data = yaml.safe_load(f) or {}
            joints = data.get("joints", data)  # tolerate a bare mapping
            idx = {n: i for i, n in enumerate(CANONICAL_JOINT_ORDER)}
            n_set = 0
            for name, lim in (joints or {}).items():
                if name not in idx or not isinstance(lim, dict):
                    continue
                lo = lim.get("lower")
                hi = lim.get("upper")
                if lo is None or hi is None:
                    continue
                lo, hi = float(lo), float(hi)
                if lo > hi:
                    lo, hi = hi, lo
                self.limit_lower[idx[name]] = lo
                self.limit_upper[idx[name]] = hi
                n_set += 1
            if n_set == 0:
                log.warn(f"limits_file '{path}' set no joints; "
                         f"using global fallback for all.")
            else:
                self.limits_source = f"{path} ({n_set} joints)"
                log.info(f"Loaded per-joint limits from {path} "
                         f"({n_set} joints overridden).")
        except Exception as exc:  # noqa: BLE001
            log.error(f"Could not load limits_file '{path}' ({exc}); "
                      f"using global joint_min/joint_max for all joints.")

    def _run_params(self) -> dict:
        """Everything about this run that is constant for its whole duration.

        Written to <run>.meta.json so a sweep of runs can be compared on what
        was actually configured rather than on what the filename claims.
        """
        return {
            "checkpoint_label": self.checkpoint_label,
            "task_description": self.task_description,
            "notes": self.run_notes,
            "control_rate_hz": self.control_rate_hz,
            "execution_horizon": self.execution_horizon,
            "prefetch_enable": self.prefetch_enable,
            "prefetch_lead": self.prefetch_lead,
            "rtc_enable": self.rtc_enable,
            "rtc_overlap_steps": self.rtc_overlap_steps,
            "rtc_frozen_steps": self.rtc_frozen_steps,
            "rtc_ramp_rate": self.rtc_ramp_rate,
            "rtc_freeze_warn_mrad": self.rtc_freeze_warn_mrad,
            "enable_left_arm": self.enable_left_arm,
            "enable_left_gripper": self.enable_left_gripper,
            "enable_right_arm": self.enable_right_arm,
            "enable_right_gripper": self.enable_right_gripper,
            "enable_limits": self.enable_limits,
            "limits_source": self.limits_source,
            "joint_min": self.joint_min,
            "joint_max": self.joint_max,
            "ood_warn_fraction": self.ood_warn_fraction,
            "gripper_max_effort": self.gripper_max_effort,
            "gripper_min_delta": self.gripper_min_delta,
            "server_host": self.server_host,
            "server_port": self.server_port,
            "zmq_timeout_ms": self.zmq_timeout_ms,
            "log_chunks": self.log_chunks,
            "chunks_file": (os.path.basename(self.data_log.chunks_path)
                            if self.data_log is not None and self.log_chunks else None),
            "package_version": PACKAGE_VERSION,
            "joint_order": list(CANONICAL_JOINT_ORDER),
        }

    def _print_startup_banner(self) -> None:
        log = self.get_logger()
        log.info("=" * 70)
        log.info("  adibot_gr00t_client :: inference_client "
                 "(BIMANUAL 16-D + RUN LOGGING)")
        log.info("=" * 70)
        # Printed because the robot computer has no git: the version is the
        # only way to tell at a glance which build is actually running after
        # a package directory is copied over by hand.
        log.info(f"  package version      : {PACKAGE_VERSION}")
        log.info(f"  GR00T server         : tcp://{self.server_host}:{self.server_port}")
        log.info(f"  control_rate_hz      : {self.control_rate_hz}")
        log.info(f"  execution_horizon    : {self.execution_horizon}")
        log.info(f"  prefetch_enable      : {self.prefetch_enable}")
        if self.prefetch_enable:
            log.info(f"  prefetch_lead        : {self.prefetch_lead} steps")
        log.info(f"  rtc_enable           : {self.rtc_enable}")
        if self.rtc_enable:
            log.info(f"    overlap/frozen/ramp: {self.rtc_overlap_steps}/"
                     f"{self.rtc_frozen_steps}/{self.rtc_ramp_rate}")
            if self.rtc_freeze_warn_mrad > 0:
                log.info(f"    freeze check       : warn above "
                         f"{self.rtc_freeze_warn_mrad:g} mrad")
            else:
                log.info("    freeze check       : DISABLED")
            log.warn("  RTC needs a PATCHED policy server (server/README.md). "
                     "Every returned chunk is checked against the seed sent "
                     "with its request; an ERROR line below means the server "
                     "ignored it.")
        log.info(f"  gripper_max_effort   : {self.gripper_max_effort}")
        log.info(f"  gripper_min_delta    : {self.gripper_min_delta}")
        log.info(f"  enable_left_arm      : {self.enable_left_arm}")
        log.info(f"  enable_left_gripper  : {self.enable_left_gripper}")
        log.info(f"  enable_right_arm     : {self.enable_right_arm}")
        log.info(f"  enable_right_gripper : {self.enable_right_gripper}")
        if self.enable_limits:
            log.info(f"  joint limits         : {self.limits_source}")
            log.info(f"    fallback range     : [{self.joint_min}, {self.joint_max}] rad")
        else:
            log.warn("  joint limits         : DISABLED (enable_limits=false) "
                     "-- commands published unchecked; e-stop is the only guard")
        log.info(f"  ood_warn_fraction    : {self.ood_warn_fraction}")
        log.info(f"  task_description     : {self.task_description!r}")
        log.info(f"  sub joint_states     : {self.joint_states_topic}")
        log.info(f"  sub head             : {self.head_image_topic}")
        log.info(f"  sub wrist_left       : {self.wrist_left_image_topic}")
        log.info(f"  sub wrist_right      : {self.wrist_right_image_topic}")
        log.info(f"  pub left arm         : {self.left_arm_command_topic}")
        log.info(f"  pub right arm        : {self.right_arm_command_topic}")
        log.info(f"  act left gripper     : {self.left_gripper_action}")
        log.info(f"  act right gripper    : {self.right_gripper_action}")
        if self.checkpoint_label:
            log.info(f"  checkpoint_label     : {self.checkpoint_label}")
        else:
            log.warn("  checkpoint_label     : (unset) -- the log will not "
                     "record which checkpoint produced this run")
        if self.data_log is not None:
            log.info(f"  run log              : {self.data_log.csv_path}")
            log.info(f"  run meta             : {self.data_log.meta_path}")
        else:
            log.info("  run log              : DISABLED")
        log.info("=" * 70)

    def start(self) -> bool:
        """Ping the server; only start the control loop if it responds."""
        log = self.get_logger()
        max_attempts = 10
        for attempt in range(1, max_attempts + 1):
            log.info(f"Pinging GR00T server ({attempt}/{max_attempts}) ...")
            ok, resp = self.client.ping()
            if ok:
                log.info(f"Ping OK -> server reachable. response={resp!r}")
                if self.prefetch_enable:
                    # Hand the ZMQ client over to the worker thread. From here
                    # on, the main thread must never touch self.client.
                    self._worker = threading.Thread(
                        target=self._inference_worker, daemon=True,
                        name="gr00t-inference")
                    self._worker.start()
                period = 1.0 / self.control_rate_hz
                self._timer = self.create_timer(period, self._control_loop)
                log.info(f"Control loop started at {self.control_rate_hz} Hz.")
                return True
            log.warn("Ping failed (timeout). Retrying in 2s ...")
            time.sleep(2.0)
        log.error(f"Server did not respond to ping after {max_attempts} "
                  f"attempts. Control loop will NOT start.")
        return False

    # ------------------------------------------------------------ subscribers
    # >>> CRITICAL SECTION 1 of 3: JOINT REORDERING ---------------------------
    def _joint_states_cb(self, msg: JointState) -> None:
        """Reorder scrambled /joint_states into the canonical 16-D vector.

        Position feeds the policy. Velocity and effort are reordered the same
        way purely for logging -- they are NOT part of the observation. Fields
        the driver omits are left as NaN so the CSV stays rectangular.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        state = np.zeros(len(CANONICAL_JOINT_ORDER), dtype=np.float32)
        for i, jname in enumerate(CANONICAL_JOINT_ORDER):
            if jname in name_to_pos:
                state[i] = float(name_to_pos[jname])
            else:
                state[i] = 0.0
                if jname not in self._missing_joint_warned:
                    self._missing_joint_warned.add(jname)
                    self.get_logger().warn(
                        f"Joint '{jname}' missing from /joint_states; using 0.0.")
        self.latest_state = state

        # Optional diagnostic channels, reordered identically (NaN when absent).
        def reorder_optional(values):
            if values is None or len(values) != len(msg.name):
                return None
            lookup = dict(zip(msg.name, values))
            out = np.full(len(CANONICAL_JOINT_ORDER), np.nan, dtype=np.float32)
            for i, jname in enumerate(CANONICAL_JOINT_ORDER):
                if jname in lookup:
                    out[i] = float(lookup[jname])
            return out

        self.latest_velocity = reorder_optional(msg.velocity)
        self.latest_effort = reorder_optional(msg.effort)
    # <<< END CRITICAL SECTION 1 ----------------------------------------------

    def _make_image_cb(self, which: str):
        """Factory: returns a callback that latches the named camera frame.
        rgb8 -> resize to 640x480 (same squash the recorder applied)."""
        def _cb(msg: Image) -> None:
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"cv_bridge ({which}) failed: {exc}")
                return
            img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
            img = np.ascontiguousarray(img, dtype=np.uint8)
            if which == "head":
                self.latest_head = img
            elif which == "wrist_left":
                self.latest_wrist_left = img
            elif which == "wrist_right":
                self.latest_wrist_right = img
        return _cb

    # ----------------------------------------------------------- observation
    def _build_observation(self) -> dict:
        """Nested observation: 3 cameras + 4 state keys + language.

        video keys:  head, wrist_left, wrist_right   (B,T,H,W,C)=(1,1,480,640,3)
        state keys:  left_arm(7), left_gripper(1), right_arm(7), right_gripper(1)
        Split from the 16-D canonical vector:
          left_arm=0:7  left_gripper=7  right_arm=8:15  right_gripper=15
        """
        def bt(img):
            return np.ascontiguousarray(img[np.newaxis, np.newaxis, ...],
                                        dtype=np.uint8)

        s = self.latest_state
        left_arm = s[LEFT_ARM_SLICE].astype(np.float32)[np.newaxis, np.newaxis, :]
        left_grip = np.array([[[s[LEFT_GRIPPER_IDX]]]], dtype=np.float32)
        right_arm = s[RIGHT_ARM_SLICE].astype(np.float32)[np.newaxis, np.newaxis, :]
        right_grip = np.array([[[s[RIGHT_GRIPPER_IDX]]]], dtype=np.float32)

        return {
            "video": {
                "head": bt(self.latest_head),
                "wrist_left": bt(self.latest_wrist_left),
                "wrist_right": bt(self.latest_wrist_right),
            },
            "state": {
                "left_arm": np.ascontiguousarray(left_arm, dtype=np.float32),
                "left_gripper": np.ascontiguousarray(left_grip, dtype=np.float32),
                "right_arm": np.ascontiguousarray(right_arm, dtype=np.float32),
                "right_gripper": np.ascontiguousarray(right_grip, dtype=np.float32),
            },
            "language": {
                "annotation.human.task_description": [[self.task_description]],
            },
        }

    # ---------------------------------------------------------- action parsing
    # >>> CRITICAL SECTION 2 of 3: ACTION PARSING -> (H, 16) ------------------
    # Server returns a horizon of predicted steps. Normalize to a clean (H, 16)
    # array with columns in canonical order:
    #   [L_arm_j1..j7, L_grip, R_arm_j1..j7, R_grip]
    #
    #   (A) dict  : {"left_arm","left_gripper","right_arm","right_gripper"}
    #   (B) array : single ndarray, last dim == 16 (canonical layout).
    def _normalize_action(self, action) -> Optional[np.ndarray]:
        try:
            if isinstance(action, list):
                if len(action) == 0:
                    raise ValueError("empty action list from server")
                action = action[0]
            if isinstance(action, dict):
                return self._normalize_action_dict(action)
            return self._normalize_action_array(np.asarray(action))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to parse action: {exc}")
            return None

    def _normalize_action_dict(self, action: dict) -> np.ndarray:
        def col(key, width):
            a = np.squeeze(np.asarray(action[key], dtype=np.float32))
            if a.ndim == 0:
                a = a.reshape(1, 1)
            elif a.ndim == 1:
                # (H,) for a 1-wide key, or (width,) for a single step
                a = a[:, np.newaxis] if width == 1 else a[np.newaxis, :]
            return a
        la = col("left_arm", 7)
        lg = col("left_gripper", 1)
        ra = col("right_arm", 7)
        rg = col("right_gripper", 1)
        horizon = max(la.shape[0], ra.shape[0], lg.shape[0], rg.shape[0])

        def fix(a, w):
            if a.shape[0] != horizon:
                a = np.broadcast_to(a, (horizon, w))
            return a
        la, lg, ra, rg = fix(la, 7), fix(lg, 1), fix(ra, 7), fix(rg, 1)
        return np.concatenate([la, lg, ra, rg], axis=-1).astype(np.float32)  # (H,16)

    def _normalize_action_array(self, arr: np.ndarray) -> np.ndarray:
        arr = arr.astype(np.float32)
        if arr.ndim == 1:
            arr = arr[np.newaxis, :]
        dim = arr.shape[-1]
        if dim != 16:
            raise ValueError(
                f"Unexpected action last-dim={dim}; expected 16 "
                f"(bimanual canonical layout). Update _normalize_action_array().")
        return arr  # already (H, 16) in canonical order
    # <<< END CRITICAL SECTION 2 ----------------------------------------------

    @staticmethod
    def _describe_action(action) -> str:
        if isinstance(action, dict):
            parts = [f"{k}: shape={np.asarray(v).shape} dtype={np.asarray(v).dtype}"
                     for k, v in action.items()]
            return "dict { " + " | ".join(parts) + " }"
        arr = np.asarray(action)
        return f"array shape={arr.shape} dtype={arr.dtype}"

    # ------------------------------------------------------------- control loop
    def _control_loop(self) -> None:
        log = self.get_logger()
        # Wait for state + all THREE cameras.
        missing = []
        if self.latest_state is None:
            missing.append("joint_states")
        if self.latest_head is None:
            missing.append("head")
        if self.latest_wrist_left is None:
            missing.append("wrist_left")
        if self.latest_wrist_right is None:
            missing.append("wrist_right")
        if missing:
            log.warn(f"Waiting for inputs: {', '.join(missing)} ...",
                     throttle_duration_sec=2.0)
            return

        if not self.prefetch_enable:
            self._control_loop_blocking()
            return

        # -- prefetch mode ----------------------------------------------------
        # 1) Ingest any completed inference response (worker thread -> here).
        self._ingest_responses()

        # 2) Fire the next request once the buffer is down to prefetch_lead
        #    steps (and no request is already in flight).
        if not self._inflight and len(self._action_buffer) <= self.prefetch_lead:
            self._fire_request()

        # 3) Execute one step per tick if available; otherwise hold pose.
        if self._action_buffer:
            self._pop_and_execute()
        else:
            self._stall_ticks += 1
            log.warn("Action buffer empty; holding pose while waiting for the "
                     "server (increase prefetch_lead if this recurs).",
                     throttle_duration_sec=2.0)

    def _control_loop_blocking(self) -> None:
        """Original v0.3.0 behaviour: request inline, stalling the loop at
        every chunk boundary. Kept for fallback via prefetch_enable:=false."""
        log = self.get_logger()
        if not self._action_buffer:
            req = self._make_request()
            self._inference_seq += 1
            self._count_request(req)

            t_req = time.time()
            try:
                action, info = self.client.get_action(req.obs, req.options)
            except zmq.error.Again:
                log.warn("ZMQ timeout on get_action; reconnecting + skipping.")
                self._failed_requests += 1
                self.client.reconnect()
                return
            except zmq.error.ZMQError as exc:
                log.warn(f"ZMQ error on get_action ({exc}); "
                         f"reconnecting + skipping.")
                self._failed_requests += 1
                self.client.reconnect()
                return
            latency_ms = (time.time() - t_req) * 1000.0
            self._accept_chunk(req, action, info, latency_ms, skip_steps=0)

        if self._action_buffer:
            self._pop_and_execute()

    # ------------------------------------------------------- prefetch plumbing
    def _make_request(self) -> _Request:
        """Build the next get_action request: the observation and -- when RTC
        is on and there is a plan to stitch onto -- the aligned seed with its
        options. Does not consume the sequence number; the caller does that
        once the request is actually sent."""
        obs = self._build_observation()
        req = _Request(seq=self._inference_seq, obs=obs, options=None,
                       req_tick=self._tick, t_req=time.time(),
                       obs_state=np.array(self.latest_state, dtype=np.float32),
                       rtc=None)
        if self.rtc_enable and self._last_raw_action is not None:
            req.rtc, req.options = self._attach_rtc_seed(obs)
        return req

    def _attach_rtc_seed(self, obs: dict
                         ) -> Tuple[Optional[dict], Optional[dict]]:
        """Put the aligned seed into `obs["action"]` and return (seed facts,
        options), or (None, None) when no seed can be sent -- the request then
        goes out as plain inference.

        The seed is the current plan from the step the arm is about to execute
        (`_chunk_cursor`): the new chunk's step 0 stands for that same
        instant, and the server lays the last `overlap` rows it is sent over
        the new chunk's first `overlap` steps (rtc.py explains the
        re-indexing). The overlap is cut to what remains of the plan.
        """
        log = self.get_logger()
        if not isinstance(self._last_raw_action, dict):
            if not self._rtc_form_warned:
                self._rtc_form_warned = True
                log.warn("rtc_enable=true but the server returns a non-dict "
                         "action; cannot send the previous chunk back. RTC "
                         "is inactive.")
            return None, None
        plan = plan_seed(self._last_raw_horizon, self._chunk_cursor,
                         self.rtc_overlap_steps, self.rtc_frozen_steps)
        if plan is None:
            # The plan is used up (cursor at or past its end, e.g. after a
            # long stall): nothing left to stitch onto.
            return None, None
        try:
            obs["action"] = build_seed_action(self._last_raw_action, plan)
        except ValueError as exc:
            if not self._rtc_form_warned:
                self._rtc_form_warned = True
                log.warn(f"cannot build the RTC seed from the server's action "
                         f"({exc}); RTC is inactive.")
            return None, None
        if plan.overlap < self.rtc_overlap_steps and not self._rtc_reduced_warned:
            self._rtc_reduced_warned = True
            log.warn(
                f"RTC overlap cut to {plan.overlap} for this request: only "
                f"{plan.overlap} steps of the current chunk remain past the "
                f"step the arm is on (cursor {plan.offset} of {plan.horizon}). "
                f"Fire earlier (raise prefetch_lead) or lower "
                f"rtc_overlap_steps. Counted in the sidecar as "
                f"rtc_reduced_overlap; warned once.")
        facts = {
            "prev_seq": self._last_seq,
            "offset": plan.offset,
            "overlap": plan.overlap,
            "frozen": plan.frozen,
            "ramp": self.rtc_ramp_rate,
            # Canonical (overlap, 16) copy of the rows sent, for the freeze
            # check on arrival and the chunk store.
            "seed": seed_rows(self._last_steps, plan),
        }
        return facts, rtc_options(plan, self.rtc_ramp_rate)

    def _count_request(self, req: _Request) -> None:
        """Sidecar bookkeeping for a request that was actually sent."""
        if req.rtc is None:
            return
        self._rtc_requests += 1
        if req.rtc["overlap"] < self.rtc_overlap_steps:
            self._rtc_reduced_overlap += 1

    def _fire_request(self) -> None:
        """Queue one inference request for the worker thread."""
        req = self._make_request()
        try:
            # The whole record rides along and comes back with the response,
            # so the receiver knows how many steps were executed while the
            # request was in flight (-> steps to skip) and exactly which seed
            # it carried.
            self._req_q.put_nowait(req)
        except queue.Full:
            return  # _inflight guard should prevent this; drop rather than block
        self._inference_seq += 1
        self._inflight = True
        self._count_request(req)

    def _ingest_responses(self) -> None:
        """Drain completed inference responses from the worker thread."""
        log = self.get_logger()
        while True:
            try:
                req, action, info, latency_ms = self._resp_q.get_nowait()
            except queue.Empty:
                return
            self._inflight = False
            if action is None:
                self._failed_requests += 1
                log.warn("Inference request failed (timeout/ZMQ error); "
                         "a new request will fire next tick.")
                continue
            # Steps executed since the request's observation was taken. The
            # chunk is anchored to that observation, so these are in the past.
            # Counted in EXECUTED steps (not wall ticks): during a stall the
            # arm holds and the state does not advance, so the anchor is not
            # stale by held ticks.
            skip = max(0, self._tick - req.req_tick)
            self._accept_chunk(req, action, info, latency_ms, skip_steps=skip)

    def _accept_chunk(self, req: _Request, action, info, latency_ms: float,
                      skip_steps: int) -> None:
        """Parse a server response and (re)fill the action buffer.

        `req` is the record of the request that produced this response. It
        travelled with the request and came back with the reply, so what it
        says was sent (seed, options, tick, state) is exact whatever happened
        in between."""
        log = self.get_logger()
        seq = req.seq
        if not self._logged_action_structure:
            self._logged_action_structure = True
            log.info("First action received. Structure: "
                     + self._describe_action(action))
            if isinstance(info, dict) and info:
                log.info(f"Action info keys: {list(info.keys())}")
            # The server's first info dict is the only channel through which
            # the client can learn anything about what is being served (model
            # path, denoising steps, ...). Store it verbatim; it is empty on
            # an unpatched server, which is itself worth recording.
            if self.data_log is not None:
                self.data_log.update_meta(
                    server_info_first=info if isinstance(info, dict) else None)

        # Unwrap the list envelope once; _normalize_action tolerates both.
        raw = action[0] if isinstance(action, list) and action else action

        steps = self._normalize_action(raw)   # (H, 16)
        if steps is None or steps.shape[0] == 0:
            self._dropped_chunks += 1
            log.warn("Action parsed to empty/invalid; dropping chunk.")
            return
        horizon = steps.shape[0]

        # RTC: compare what came back with the seed this request carried.
        rtc_facts = None
        if req.rtc is not None:
            rtc_facts = dict(req.rtc)
            rtc_facts["freeze_err_mrad"] = freeze_error_mrad(
                steps, req.rtc["seed"], req.rtc["frozen"])
            rtc_facts["server_ack"] = self._server_rtc_ack(info)
            self._check_rtc_effect(seq, rtc_facts, skip_steps)

        # Record the chunk in full BEFORE any accept/drop decision, so the
        # unexecuted tail and even entirely-stale chunks are preserved for
        # offline overlap analysis. The CSV only ever carries executed steps.
        if self.data_log is not None and self.log_chunks:
            self.data_log.log_chunk(
                seq, steps, skip_steps,
                t_req_wall=req.t_req, req_tick=req.req_tick,
                obs_state=req.obs_state, rtc=rtc_facts)

        # Record the server's true chunk length once, so the sidecar carries
        # it even if the run executes only part of every chunk. Guarded by a
        # dedicated flag: _pending_chunk_len is cleared on every logged tick,
        # so using it here rewrote the whole sidecar on every chunk -- a
        # synchronous file write in the 30 Hz control loop.
        if self.data_log is not None and not self._chunk_len_logged:
            self._chunk_len_logged = True
            self.data_log.update_meta(server_chunk_len=horizon)

        if horizon < self.execution_horizon and not self._short_chunk_warned:
            self._short_chunk_warned = True
            log.warn(
                f"Server chunk is {horizon} steps but execution_horizon="
                f"{self.execution_horizon}; executing {horizon}. The chunk "
                f"length is set by the served checkpoint's modality config "
                f"(action delta_indices), not by this parameter.")

        if skip_steps >= horizon:
            self._stale_chunks += 1
            # The plan is deliberately left as it was: this chunk never runs,
            # so seeding the next request from it would grow the next chunk
            # out of motion the arm never performed. (v0.4.0-v0.6.0 updated
            # the seed source above this check -- the stale-chunk RTC seed
            # bug of CHANGELOG v0.5.0.) Reachable only when the round trip
            # exceeds a whole chunk (~1.3 s at 40 steps), i.e. a network or
            # server stall; the stale_chunks count in the sidecar says
            # whether a given run ever hit it.
            log.warn(f"Entire {horizon}-step chunk is stale ({skip_steps} "
                     f"steps executed since its observation); dropping it.")
            return

        end = min(skip_steps + self.execution_horizon, horizon)
        self._current_inference_seq = seq
        # Replace (not append): the new chunk is anchored to a fresher
        # observation, so it supersedes whatever remains of the old one.
        self._action_buffer = [(i, steps[i]) for i in range(skip_steps, end)]
        # This chunk is now the plan: the seed source for the next request,
        # with the cursor on its first step still to be executed.
        self._last_raw_action = raw
        self._last_steps = steps
        self._last_raw_horizon = horizon
        self._last_seq = seq
        self._chunk_cursor = skip_steps
        self._pending_latency_ms = latency_ms
        self._pending_chunk_len = horizon
        self._pending_skip = skip_steps
        self._pending_rtc = req.rtc is not None
        log.debug(f"chunk accepted: horizon={horizon}, skipped {skip_steps}, "
                  f"buffered {len(self._action_buffer)}, "
                  f"latency={latency_ms:.1f}ms")

    @staticmethod
    def _server_rtc_ack(info) -> int:
        """1/0 when the server's info dict says whether it applied RTC (the
        patch in server/ adds that key); -1 when it says nothing."""
        if isinstance(info, dict) and "rtc_applied" in info:
            return 1 if info["rtc_applied"] else 0
        return -1

    def _check_rtc_effect(self, seq: int, facts: dict, skip_steps: int) -> None:
        """Did the server honour the seed, and did the chunk arrive while its
        frozen block was still ahead of the arm? The first is an error printed
        once; both are counted for the sidecar and logged per chunk."""
        log = self.get_logger()
        err = facts["freeze_err_mrad"]
        thr = self.rtc_freeze_warn_mrad
        if thr > 0 and np.isfinite(err) and err > thr:
            self._rtc_freeze_violations += 1
            if not self._rtc_freeze_warned:
                self._rtc_freeze_warned = True
                log.error(
                    f"RTC is NOT taking effect: chunk {seq}'s first "
                    f"{facts['frozen']} steps differ from the seed sent by up "
                    f"to {err:.1f} mrad (limit {thr:g}). The server is "
                    f"ignoring observation['action'] -- unpatched "
                    f"gr00t_policy.py, old code on its import path, or not "
                    f"restarted (see server/README.md). Counted in the "
                    f"sidecar as rtc_freeze_violations; warned once.")
        if facts["server_ack"] == -1 and not self._rtc_ack_noted:
            self._rtc_ack_noted = True
            log.info("server info carries no 'rtc_applied' key, so the server "
                     "does not confirm RTC itself (patch without the info "
                     "dict?); the per-chunk freeze check covers it.")
        if skip_steps > facts["frozen"]:
            self._rtc_frozen_overrun += 1
            log.warn(f"chunk {seq} arrived after its frozen block had played "
                     f"out ({skip_steps} steps executed in flight, "
                     f"{facts['frozen']} frozen); the seam falls in the ramp "
                     f"or fresh part. Raise rtc_frozen_steps toward the "
                     f"round trip in ticks.", throttle_duration_sec=10.0)

    def _inference_worker(self) -> None:
        """Blocking ZMQ round trips, off the control thread. Sole owner of
        self.client after start(); exits on a None sentinel."""
        while True:
            req = self._req_q.get()
            if req is None:
                return
            obs, req.obs = req.obs, None   # the images need not outlive the send
            t0 = time.time()
            try:
                action, info = self.client.get_action(obs, req.options)
            except zmq.error.Again:
                self.client.reconnect()
                self._resp_q.put((req, None, None, 0.0))
                continue
            except zmq.error.ZMQError as exc:
                self.get_logger().warn(
                    f"ZMQ error in inference worker ({exc}); reconnecting.")
                self.client.reconnect()
                self._resp_q.put((req, None, None, 0.0))
                continue
            latency_ms = (time.time() - t0) * 1000.0
            self._resp_q.put((req, action, info, latency_ms))

    def _pop_and_execute(self) -> None:
        """Take the next buffered step, move the plan cursor past it, and
        execute it. A request fires BEFORE this on the same tick and reads
        the cursor, so the cursor names the step executed at the observation
        instant -- the step the new chunk's step 0 stands for."""
        horizon_idx, step = self._action_buffer.pop(0)   # (16,)
        self._chunk_cursor = horizon_idx + 1
        self._execute_step(step, horizon_idx)

    def _execute_step(self, step: np.ndarray, horizon_idx: int = -1) -> None:
        """Split the 16-D step into sides; validate limits; publish per side."""
        log = self.get_logger()
        self._tick += 1
        left_arm = step[LEFT_ARM_SLICE].astype(np.float64)
        left_grip = float(step[LEFT_GRIPPER_IDX])
        right_arm = step[RIGHT_ARM_SLICE].astype(np.float64)
        right_grip = float(step[RIGHT_GRIPPER_IDX])

        def violates(arm_cmd, base_idx, label, warn):
            """Detect a per-joint soft-limit breach. Each of the 7 arm joints is
            checked against its own [lower, upper] (canonical index base_idx+i).
            Always evaluated so the log records violations even for disabled
            sides, but only warns for enabled sides -- a parked arm should not
            spam the console.

            When enable_limits is False the guard is off entirely: no joint ever
            counts as violating, so every commanded value is published unchecked
            (hardware e-stop is the only backstop)."""
            if not self.enable_limits:
                return False
            out = []
            for i, v in enumerate(arm_cmd):
                ci = base_idx + i
                if v < self.limit_lower[ci] or v > self.limit_upper[ci]:
                    out.append((i, v, self.limit_lower[ci], self.limit_upper[ci]))
            if out and warn:
                details = ", ".join(
                    f"j{i + 1}={v:.3f} not in [{lo:.3f},{hi:.3f}]"
                    for i, v, lo, hi in out)
                log.warn(f"{label} out of soft limits ({details}); "
                         f"SKIPPING this side.")
            return bool(out)

        # LEFT arm joints are canonical 0..6; RIGHT arm joints are 8..14.
        left_violation = violates(left_arm, 0, "left arm", self.enable_left_arm)
        right_violation = violates(right_arm, 8, "right arm",
                                   self.enable_right_arm)
        left_published = right_published = False
        # Gripper values actually commanded (post-scale); NaN when not published.
        left_grip_scaled = right_grip_scaled = float("nan")

        # LEFT side (task arm)
        if self.enable_left_arm and not left_violation:
            self._publish_arm(self.left_arm_pub, self.left_arm_command_topic,
                              left_arm)
            left_published = True
            if self.enable_left_gripper:
                left_grip_scaled = self._send_gripper_goal(
                    "left", self.left_gripper_client, left_grip)

        # RIGHT side
        if self.enable_right_arm and not right_violation:
            self._publish_arm(self.right_arm_pub, self.right_arm_command_topic,
                              right_arm)
            right_published = True
            if self.enable_right_gripper:
                right_grip_scaled = self._send_gripper_goal(
                    "right", self.right_gripper_client, right_grip)

        # Record actual-vs-commanded for this tick. Guarded so a logging fault
        # can never interrupt control. Latency belongs to the batch, so it is
        # written only on the batch's first step and then cleared.
        if self.data_log is not None:
            latency_ms = self._pending_latency_ms
            chunk_len = self._pending_chunk_len
            skip_steps = self._pending_skip
            rtc_applied = self._pending_rtc
            self._pending_latency_ms = None
            self._pending_chunk_len = None
            self._pending_skip = None
            self._pending_rtc = None
            try:
                self.data_log.log_step(
                    tick=self._tick,
                    inference_seq=self._current_inference_seq,
                    horizon_idx=horizon_idx,
                    actual_pos=self.latest_state,
                    actual_vel=self.latest_velocity,
                    actual_eff=self.latest_effort,
                    cmd=step,
                    left_grip_scaled=left_grip_scaled,
                    right_grip_scaled=right_grip_scaled,
                    left_arm_published=left_published,
                    right_arm_published=right_published,
                    left_limit_violation=left_violation,
                    right_limit_violation=right_violation,
                    latency_ms=latency_ms,
                    chunk_len=chunk_len,
                    skip_steps=skip_steps,
                    rtc_applied=rtc_applied,
                    # Steps left after this one -- 0 here means the next tick
                    # starves unless a chunk lands first.
                    buffer_len=len(self._action_buffer),
                )
            except Exception as exc:  # noqa: BLE001
                log.warn(f"data logger (step) failed: {exc}",
                         throttle_duration_sec=10.0)

        # Out-of-distribution monitor. A step counts as skipped when a side that
        # was ENABLED got rejected by the limit guard. A sustained high skip
        # rate means the policy is commanding outside its trained envelope
        # (exactly what a diverged checkpoint does) -- surface it clearly rather
        # than leaving the operator wondering why the arm froze.
        self._update_ood_monitor(
            skipped=((self.enable_left_arm and left_violation) or
                     (self.enable_right_arm and right_violation)))

    def _update_ood_monitor(self, skipped: bool) -> None:
        if self.ood_warn_fraction <= 0.0:
            return
        self._skip_window.append(1 if skipped else 0)
        # Only assess once the window is full, and warn at most every 100 ticks.
        if len(self._skip_window) < self._skip_window.maxlen:
            return
        frac = sum(self._skip_window) / len(self._skip_window)
        if frac >= self.ood_warn_fraction and \
                (self._tick - self._last_ood_warn_tick) >= 100:
            self._last_ood_warn_tick = self._tick
            self.get_logger().warn(
                f"OUT-OF-DISTRIBUTION: {frac * 100:.0f}% of the last "
                f"{len(self._skip_window)} commands exceeded soft limits and "
                f"were skipped. The policy is likely diverging from its trained "
                f"range -- check the checkpoint.")

    def _publish_arm(self, pub, topic, arm_cmd: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in arm_cmd]   # 7 joint positions
        pub.publish(msg)
        self.get_logger().debug(
            f"published arm -> {topic}: {np.array2string(arm_cmd, precision=3)}")

    # >>> CRITICAL SECTION 3 of 3: GRIPPER COMMANDING -------------------------
    def _send_gripper_goal(self, side: str, client: ActionClient,
                           gripper_val: float) -> float:
        """Command a gripper via its GripperCommand action server.

        Verified on hardware (2026-07): the grippers are driven by ros2_control
        `position_controllers/GripperActionController` -- an ACTION interface.
        GripperCommand.position is the finger joint position in the SAME units
        as /joint_states and the dataset (~0.0 = closed, ~0.05 = open), so the
        VLA's gripper output passes through RAW -- no scaling. max_effort caps
        grip force (10.0 verified: brisk motion, gentle stall).

        Goals are sent fire-and-forget (send_goal_async, result ignored) so the
        control loop never blocks, and only when the command moved more than
        gripper_min_delta since the last goal -- the controller preempts the
        previous goal on each new one, so intermediate values are disposable.

        Returns the position actually commanded this tick (for the run log),
        even on ticks where no new goal was sent because the value was static.
        """
        pos = float(gripper_val)
        last = self._last_gripper_sent[side]
        if last is not None and abs(pos - last) < self.gripper_min_delta:
            return pos                       # unchanged; nothing to send
        if not client.server_is_ready():
            if side not in self._gripper_server_warned:
                self._gripper_server_warned.add(side)
                self.get_logger().warn(
                    f"{side} gripper action server not available; gripper "
                    f"commands are being dropped (will warn once).")
            return float("nan")
        self._gripper_server_warned.discard(side)
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = self.gripper_max_effort
        client.send_goal_async(goal)         # fire-and-forget
        self._last_gripper_sent[side] = pos
        self.get_logger().debug(
            f"gripper goal ({side}): position={pos:.4f} "
            f"max_effort={self.gripper_max_effort}")
        return pos
    # <<< END CRITICAL SECTION 3 ----------------------------------------------

    def destroy_node(self):
        # Close the run log first so the CSV is flushed even on Ctrl-C.
        if self.data_log is not None:
            try:
                # Outcome counts the CSV cannot carry: a request that never
                # produced an executed step leaves no row, only a gap in the
                # inference_seq numbering. Recording them here makes such a
                # gap attributable instead of unexplained.
                self.data_log.update_meta(
                    stall_ticks=self._stall_ticks,
                    failed_requests=self._failed_requests,
                    dropped_chunks=self._dropped_chunks,
                    stale_chunks=self._stale_chunks,
                    requests_issued=self._inference_seq,
                    rtc_requests=self._rtc_requests,
                    rtc_reduced_overlap=self._rtc_reduced_overlap,
                    rtc_freeze_violations=self._rtc_freeze_violations,
                    rtc_frozen_overrun=self._rtc_frozen_overrun,
                )
            except Exception:  # noqa: BLE001
                pass
            try:
                self.data_log.close()
            except Exception:  # noqa: BLE001
                pass
        # Stop the inference worker (owner of the ZMQ socket) before closing
        # the client underneath it.
        if self._worker is not None and self._worker.is_alive():
            try:
                self._req_q.put_nowait(None)
            except Exception:  # noqa: BLE001
                pass
            self._worker.join(timeout=2.0)
        try:
            self.client.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InferenceClientNode()
    try:
        if node.start():            # only spin if ping succeeded
            rclpy.spin(node)
        else:
            node.get_logger().error("Refusing to run control loop; shutting down.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
