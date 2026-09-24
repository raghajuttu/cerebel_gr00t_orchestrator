"""The orchestrator: one timer, one mission, one thing moving at a time.

Everything the robot does autonomously is a step in a mission file. This node
reads that file, asks ``MissionRunner`` what comes next, and carries it out:

    navigate    -> Nav2 NavigateToPose            (base moves, arms parked)
    run_policy  -> a GR00T inference client       (arms move, base gated shut)
    park_arms   -> a ramp to a known pose         (arms move, base gated shut)
    check_arms  -> verify the arms are inside a named envelope (nothing moves)
    wait        -> nothing at all

Three properties are deliberate:

**Nothing blocks.** Every handler is poll-shaped and the whole mission advances
inside one timer callback, so the e-stop subscription and the service calls are
always live. The one exception is stopping a policy, which blocks until the child
process is gone -- by design, because the base must not start moving while
another process is still publishing arm commands.

**The base is gated by a heartbeat.** ``base_enable`` is published true only
while the current step is a navigate, and ``base_adapter`` stops the wheels if it
stops hearing it. The interlock therefore survives this node crashing, hanging or
being killed -- it does not depend on the mission logic being correct.

**A phase ends for a stated reason.** Every step reports ``(ok, reason)`` and the
reasons are what the summary is made of, so a mission that fails says what the
robot was doing and what it was waiting for.
"""

from __future__ import annotations

import json
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from .arm_hold import ArmHold
from .arm_park import ArmParker, load_park_poses
from .base_move import BaseMoveConfig, BaseMover
from .envelopes import load_envelopes
from .joints import reorder_by_name
from .mission import Mission, MissionError, navigate_safety_warnings
from .nav_client import NAV2_MISSING, NavClient
from .phase_monitor import MonitorConfig, PhaseMonitor
from .policy_preflight import failures, preflight, summary
from .signal_source import SignalSource
from .policy_runner import (
    PolicyPrepareError,
    PolicyRunner,
    RunnerConfig,
    describe_command,
)
from .state_machine import Action, MissionRunner, Phase

JOINT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("auto_start", False)
        # Two loops, two rates. control_rate_hz sustains motion (the park ramp)
        # and the base-enable heartbeat; supervisor_rate_hz decides when a step
        # is done and switches to the next. See docs/ARCHITECTURE.md.
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("supervisor_rate_hz", 10.0)
        self.declare_parameter("shutdown_when_done", True)
        # The two halves of a rehearsal, separately: mock_policy starts no
        # inference client, mock_nav sends no Nav2 goal. The desk test mocks the
        # policy but keeps navigation real, because a fake base that never
        # receives a goal proves nothing about the Nav2 configuration.
        self.declare_parameter("mock_policy", False)
        self.declare_parameter("mock_nav", False)

        # Navigation
        # move_base: the wheel-arc stand-in for Nav2. See base_move.py for why
        # Nav2 cannot close its loop on this chassis yet.
        self.declare_parameter("wheel_rpm_topic", "/all_wheel_rpm")
        self.declare_parameter("base_cmd_out_topic", "/cmd_vel")
        self.declare_parameter("move_speed_mps", 0.10)
        self.declare_parameter("move_min_speed_mps", 0.03)
        self.declare_parameter("move_taper_m", 0.15)
        self.declare_parameter("move_scale_lateral", 1.05)
        self.declare_parameter("move_scale_axial", 1.02)
        self.declare_parameter("move_tolerance_m", 0.01)
        self.declare_parameter("move_timeout_s", 60.0)

        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("nav_timeout_s", 120.0)
        self.declare_parameter("nav_server_wait_s", 20.0)

        # Policies
        self.declare_parameter(
            "policy_cmd", ["ros2", "run", "adibot_gr00t_client", "inference_client"]
        )
        self.declare_parameter("log_dir", "~/adibot_logs")
        self.declare_parameter("policy_startup_grace_s", 20.0)
        # Some robots need the arm controller re-initialised between clients.
        # Empty disables it. Split as a list: ["ros2", "control", ...].
        self.declare_parameter("policy_prepare_cmd", [""])
        self.declare_parameter("policy_prepare_timeout_s", 15.0)
        self.declare_parameter("policy_prepare_settle_s", 0.0)
        # Ping every policy server once, at mission start, before anything moves.
        self.declare_parameter("policy_preflight", True)
        self.declare_parameter("preflight_timeout_ms", 5000)

        # Phase termination -- measure these off a real run before trusting them
        self.declare_parameter("grasp_close_m", 0.010)
        self.declare_parameter("grasp_open_m", 0.030)
        self.declare_parameter("motion_eps", 0.05)  # rad/s, a speed
        self.declare_parameter("settle_grace_s", 3.0)
        self.declare_parameter("stale_state_s", 1.0)

        # Parking
        self.declare_parameter("enable_park", False)
        self.declare_parameter("park_poses_file", "")
        self.declare_parameter("max_joint_speed", 0.15)
        self.declare_parameter("park_tolerance", 0.01)
        self.declare_parameter("park_timeout_s", 60.0)
        self.declare_parameter("gripper_max_effort", 10.0)
        self.declare_parameter("command_grippers", True)

        # Holding the arms across a policy switch. The pick ends with the object
        # in the gripper and the place policy starts from there, so the seconds
        # while a new client is starting up must not be seconds with nobody
        # commanding the arms. See arm_hold.py.
        self.declare_parameter("hold_arms_between_policies", True)
        self.declare_parameter("hold_quiet_s", 0.25)
        self.declare_parameter(
            "left_arm_command_topic", "/left_forward_position_controller/commands"
        )
        self.declare_parameter(
            "right_arm_command_topic", "/right_forward_position_controller/commands"
        )

        # Envelopes for check_arms -- the measurement that backs up a policy's
        # ends_parked claim before the base is allowed to move.
        self.declare_parameter("arm_envelopes_file", "")

        # Interfaces
        self.declare_parameter("estop_topic", "/orchestrator/estop")
        self.declare_parameter("base_enable_topic", "/orchestrator/base_enable")
        self.declare_parameter("status_topic", "/orchestrator/status")

        get = self.get_parameter
        self.mock_policy = bool(get("mock_policy").value)
        self.mock_nav = bool(get("mock_nav").value)
        self.nav_timeout_s = float(get("nav_timeout_s").value)
        self.park_timeout_s = float(get("park_timeout_s").value)
        self.policy_startup_grace_s = float(get("policy_startup_grace_s").value)
        self.policy_preflight = bool(get("policy_preflight").value)
        self.preflight_timeout_ms = int(get("preflight_timeout_ms").value)
        self.shutdown_when_done = bool(get("shutdown_when_done").value)

        self.monitor_config = MonitorConfig(
            grasp_close_m=float(get("grasp_close_m").value),
            grasp_open_m=float(get("grasp_open_m").value),
            motion_eps=float(get("motion_eps").value),
            settle_grace_s=float(get("settle_grace_s").value),
            stale_state_s=float(get("stale_state_s").value),
            # A phase that has just started gets the client's warm-up time
            # before a gap in /joint_states counts against it.
            stale_grace_s=self.policy_startup_grace_s,
        )
        self.monitor_config.validate()

        self.mission = self._load_mission(str(get("mission_file").value))
        self.envelopes = load_envelopes(str(get("arm_envelopes_file").value))
        self._check_mission_envelopes()
        self.runner = MissionRunner(self.mission)

        self.move_timeout_s = float(get("move_timeout_s").value)
        self.mover = BaseMover(
            BaseMoveConfig(
                speed_mps=float(get("move_speed_mps").value),
                min_speed_mps=float(get("move_min_speed_mps").value),
                taper_m=float(get("move_taper_m").value),
                scale_lateral=float(get("move_scale_lateral").value),
                scale_axial=float(get("move_scale_axial").value),
                tolerance_m=float(get("move_tolerance_m").value),
            )
        )
        self.nav = NavClient(self, str(get("nav_action").value))
        self.policies = PolicyRunner(
            RunnerConfig(
                policy_cmd=[str(item) for item in get("policy_cmd").value],
                log_dir=str(get("log_dir").value),
                mock=self.mock_policy,
                startup_grace_s=self.policy_startup_grace_s,
                # [""] is how an empty string-array parameter arrives; strip it
                # so "not configured" and "configured with nothing" are one case.
                prepare_cmd=[
                    str(item)
                    for item in get("policy_prepare_cmd").value
                    if str(item).strip()
                ],
                prepare_timeout_s=float(get("policy_prepare_timeout_s").value),
                prepare_settle_s=float(get("policy_prepare_settle_s").value),
            ),
            self.get_logger(),
        )
        self.parker = ArmParker(
            self,
            poses=load_park_poses(str(get("park_poses_file").value)),
            enable=bool(get("enable_park").value),
            max_joint_speed=float(get("max_joint_speed").value),
            gripper_max_effort=float(get("gripper_max_effort").value),
            tolerance=float(get("park_tolerance").value),
            command_grippers=bool(get("command_grippers").value),
        )
        self._check_mission_park_profiles()

        self.hold_arms = bool(get("hold_arms_between_policies").value)
        self.holder = ArmHold(foreign_quiet_s=float(get("hold_quiet_s").value))
        left_cmd = str(get("left_arm_command_topic").value)
        right_cmd = str(get("right_arm_command_topic").value)
        self._hold_pubs = {
            "left": self.create_publisher(Float64MultiArray, left_cmd, 10),
            "right": self.create_publisher(Float64MultiArray, right_cmd, 10),
        }
        for side, topic in (("left", left_cmd), ("right", right_cmd)):
            self.create_subscription(
                Float64MultiArray,
                topic,
                lambda msg, side=side: self._on_arm_command(side, msg),
                10,
            )

        # One subscription per declared signal. Routed to a phase's monitor
        # only while that phase is running, so a reading that arrives between
        # phases is dropped rather than latched into the next one.
        self.signals = {
            name: SignalSource(self, signal)
            for name, signal in self.mission.signals.items()
        }
        for name, signal in self.mission.signals.items():
            self.get_logger().info(
                f"signal {name!r}: {signal.topic} ({signal.type}, {signal.mode})"
            )

        self._positions: Optional[List[float]] = None
        self._state_seen = False
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, JOINT_STATE_QOS
        )
        self.create_subscription(
            Float64MultiArray, str(get("wheel_rpm_topic").value), self._on_wheel_rpm, 10
        )
        self.create_subscription(Bool, str(get("estop_topic").value), self._on_estop, 10)
        self._base_enable_pub = self.create_publisher(
            Bool, str(get("base_enable_topic").value), 10
        )
        self._status_pub = self.create_publisher(String, str(get("status_topic").value), 10)
        # move_base publishes where Nav2 would, so base_adapter's interlock,
        # clamps and crab trim apply identically whichever one is driving.
        self._base_cmd_pub = self.create_publisher(
            Twist, str(get("base_cmd_out_topic").value), 10
        )

        for name, handler in (
            ("start", self._srv_start),
            ("hold", self._srv_hold),
            ("resume", self._srv_resume),
            ("abort", self._srv_abort),
            ("advance", self._srv_advance),
            ("skip", self._srv_skip),
        ):
            self.create_service(Trigger, f"~/{name}", handler)

        self._current: Optional[Action] = None
        # Where the base is on the move_base axis, in centimetres. Seeded from
        # the mission's declared start and then advanced by what each move
        # ACHIEVED, not by what it was asked for -- so a short move leaves the
        # next one longer rather than shifting every position after it.
        self._axis_cm: Optional[float] = None
        self._wheel_rpm_seen = False
        # When a policy phase ended, so the gap to the NEXT client's first arm
        # command can be reported. That gap is the switch, as the arm sees it.
        self._switch_began: Optional[float] = None
        self._monitor: Optional[PhaseMonitor] = None
        self._phase_started: float = 0.0
        self._estop = False
        self._nav_ready = False
        self._summary_printed = False

        self._print_banner()
        if bool(get("auto_start").value):
            self._begin_mission("auto_start")

        self.create_timer(
            1.0 / max(float(get("control_rate_hz").value), 1.0), self._execution_tick
        )
        self.create_timer(
            1.0 / max(float(get("supervisor_rate_hz").value), 1.0), self._supervisor_tick
        )

    # -- startup -------------------------------------------------------------

    def _load_mission(self, path: str) -> Mission:
        if not path:
            raise MissionError("mission_file parameter is empty -- nothing to run")
        mission = Mission.load(path)
        return mission

    def _check_mission_envelopes(self) -> None:
        """Fail at startup, not three steps into the mission."""
        wanted = {
            step.envelope
            for step in self.mission.steps
            if step.kind == "check_arms" and step.envelope
        }
        wanted |= {
            step.until.envelope
            for step in self.mission.steps
            if step.kind == "run_policy"
            and step.until is not None
            and step.until.envelope
        }
        missing = sorted(name for name in wanted if name not in self.envelopes)
        if missing:
            raise MissionError(
                f"the mission names envelope(s) {missing} that are not in "
                f"arm_envelopes_file (have {sorted(self.envelopes) or 'none'})"
            )



    def _check_mission_park_profiles(self) -> None:
        """Every park_arms step must name a profile that exists.

        Separate from the envelope check because it needs the parker, which is
        built later -- and calling it too early is exactly the bug this
        replaced, an AttributeError before the node could even start.

        Checked even when enable_park is false: a missing profile makes the step
        FAIL rather than be skipped, and that failure lands mid-mission with the
        arms in the air instead of here.
        """
        profiles = {
            step.profile
            for step in self.mission.steps
            if step.kind == "park_arms" and step.profile
        }
        unknown = sorted(name for name in profiles if name not in self.parker.poses)
        if unknown:
            raise MissionError(
                f"the mission names park profile(s) {unknown} that are not in "
                f"park_poses_file (have {sorted(self.parker.poses) or 'none'})"
            )

    def _print_banner(self) -> None:
        log = self.get_logger()
        log.info(f"=== cerebel orchestrator | mission {self.mission.name} ===")
        log.info(f"  file frame        : {self.mission.frame_id}")
        log.info(f"  cycles            : {self.mission.repeat}")
        log.info(f"  mock policy       : {self.mock_policy}")
        log.info(f"  mock navigation   : {self.mock_nav}")
        log.info(f"  park enabled      : {self.parker.enable}")
        for name, station in sorted(self.mission.stations.items()):
            log.info(f"  station {name:<10}: x={station.x:+.3f} y={station.y:+.3f} yaw={station.yaw:+.3f}")
        for address, names in sorted(self.mission.policy_ports().items()):
            note = " (ONE checkpoint, several prompts)" if len(names) > 1 else ""
            log.info(f"  server {address:<22}: {', '.join(sorted(names))}{note}")
        for step in self.mission.steps:
            log.info(f"  [{step.index}] {step.describe()}")
        for policy in self.mission.policies.values():
            log.info(
                f"  policy {policy.name} would run: "
                f"{describe_command(policy, 'RUNLABEL', self.policies.config)}"
            )
        for warning in navigate_safety_warnings(self.mission):
            log.warning(f"  mission lint: {warning}")
        for name in sorted(self.envelopes):
            envelope = self.envelopes[name]
            log.info(
                f"  envelope {name:<9}: {len(envelope.limits)} joints constrained"
                + (f" -- {envelope.description}" if envelope.description else "")
            )
        if not self.parker.enable:
            log.warning(
                "  park is disabled: park_arms steps report success without moving. "
                "The base will navigate with the arms wherever the policy left them."
            )

    def _begin_mission(self, why: str) -> None:
        if self.runner.terminal:
            return
        if not self._preflight_ok():
            return
        if not self._nav_ready and self._needs_nav():
            wait = float(self.get_parameter("nav_server_wait_s").value)
            if not self.nav.available:
                self.runner.fault(NAV2_MISSING)
                return
            self._nav_ready = self.nav.server_ready(wait)
            if not self._nav_ready:
                self.runner.fault(
                    f"Nav2 action server {self.nav.action_name!r} did not appear "
                    f"within {wait:.0f}s -- is the navigation stack up?"
                )
                return
        self._seed_axis()
        self.get_logger().info(f"mission start ({why})")
        self.runner.start()

    def _seed_axis(self) -> None:
        """Anchor the move_base axis where the mission says the robot is.

        This is an assertion about the world, not a measurement -- nothing on
        this chassis can tell us where it is standing. Re-seeding on every start
        is deliberate: after an abort and a manual push back to the start, the
        operator's mental model and the orchestrator's agree again.
        """
        if self.mission.start_position is None:
            self._axis_cm = None
            return
        start = self.mission.positions[self.mission.start_position]
        self._axis_cm = start.axis_cm
        self.get_logger().info(
            f"base axis anchored at {start.name!r} ({start.axis_cm:+.1f} cm) "
            "-- ASSUMED, not measured"
        )

    def _preflight_ok(self) -> bool:
        """Prove the policy server answers before the robot commits to anything.

        A mission with no policy phases skips this. So does ``mock_policy``,
        which has no server to reach.
        """
        if not self.policy_preflight or self.mock_policy or not self.mission.policies:
            return True
        results = preflight(
            self.mission.policy_ports(), self.preflight_timeout_ms, self.get_logger()
        )
        if not failures(results):
            return True
        self.runner.fault(f"policy server preflight failed -- {summary(results)}")
        return False

    def _needs_nav(self) -> bool:
        if self.mock_nav:
            return False
        return any(step.kind == "navigate" for step in self.mission.steps)

    # -- subscriptions -------------------------------------------------------

    def _on_arm_command(self, side: str, msg: Float64MultiArray) -> None:
        """Feed the hold, and time the switch.

        The first command a new client publishes is the moment the arm is
        someone's responsibility again, so the gap from the previous phase
        ending to here is the switch as the arm experiences it -- not as the
        process table sees it.
        """
        spoke_before = self.holder.client_spoke
        self.holder.observe(side, msg.data, self._now())
        if (
            self._switch_began is not None
            and not spoke_before
            and self.holder.client_spoke
        ):
            self.get_logger().info(
                f"    switch took {self._now() - self._switch_began:.2f}s "
                "from the last phase ending to the new client's first command"
            )
            self._switch_began = None

    def _on_wheel_rpm(self, msg: Float64MultiArray) -> None:
        """Fold one wheel sample into the mover. Decides nothing.

        Integrating here rather than in a loop is the same discipline as the
        joint-state callback: the arc is a rate times an interval, and the
        interval has to be the chassis's, not a timer's.
        """
        self._wheel_rpm_seen = True
        self.mover.observe(self._now(), list(msg.data))

    def _on_joint_states(self, msg: JointState) -> None:
        ordered = reorder_by_name(msg.name, msg.position)
        if ordered is None:
            if not self._state_seen:
                self.get_logger().warning(
                    "/joint_states is missing canonical joints -- check the arm bringup",
                    throttle_duration_sec=5.0,
                )
            return
        self._positions = ordered
        if not self._state_seen:
            self._state_seen = True
            self.get_logger().info("/joint_states: all 16 canonical joints present")
        # The completion check is fed here, at the rate the robot publishes,
        # rather than sampled by the supervisor loop. That is what makes
        # motion_eps a joint speed instead of "how far a joint moved between two
        # of the supervisor's ticks", which would change meaning whenever either
        # rate changed.
        if self._monitor is not None:
            self._monitor.observe(self._now(), ordered)
            # Effort rides along on the same message. Many drivers publish an
            # empty array here, so the monitor treats it as optional and a
            # mission that asks for an effort condition finds out at its
            # timeout, by name, rather than silently never firing.
            self._monitor.observe_effort(reorder_by_name(msg.name, msg.effort))

    def _on_estop(self, msg: Bool) -> None:
        asserted = bool(msg.data)
        if asserted and not self._estop:
            self.get_logger().error("E-STOP asserted -- stopping everything")
            self._estop = True
            self._stop_activity("e-stop")
            self.runner.hold("e-stop asserted")
        elif not asserted and self._estop:
            self.get_logger().warning(
                "e-stop cleared -- the current step will restart from the beginning"
            )
            self._estop = False
            self.runner.resume()

    # -- loop 1: execution ---------------------------------------------------

    def _execution_tick(self) -> None:
        """Sustain whatever is currently running. Fast, and decides nothing.

        Most steps need nothing here, because the thing doing the work has its
        own loop: Nav2 runs its controller at 20 Hz and the inference client its
        control loop at 30 Hz. This loop exists for the two things the
        orchestrator itself has to keep doing at a steady rate -- publishing the
        base-enable heartbeat, and stepping a park ramp.
        """
        self._publish_gate()
        self._hold_arms()
        if self._current is None or self.runner.held or self._estop:
            return
        if self._current.kind == "park_arms":
            self.parker.step()
        elif self._current.kind == "move_base":
            self._step_move()

    def _hold_arms(self) -> None:
        """Keep the last commanded pose on the wire while nobody else is.

        The gap this covers is the policy switch: the old client is gone, the
        new one is still pinging the server, and the arms are holding an object
        that the next policy has to start from. See arm_hold.py.

        The park ramp is the one thing that must never be competed with -- it is
        deliberately moving the arms somewhere else -- so a park phase stands
        the hold down outright rather than relying on the quiet window.
        """
        if not self.hold_arms:
            return
        if self._estop:
            # docs/SAFETY.md: the software e-stop "stops this node's commanding".
            # Republishing a held pose is still commanding, even though the value
            # does not change -- the controller latches it anyway, so the arm
            # stays exactly where it is either way. Stopping outright is what the
            # document promises and costs nothing.
            return
        kind = self._current.kind if self._current is not None else None
        # A park ramp owns the arms outright. And once a policy's client has
        # published, the rest of that phase is its own -- including its
        # silences, which are routine: the client "holds pose" at a stall or a
        # blocking chunk boundary by publishing nothing at all, for the whole
        # round trip. Stepping into those would be competing with a policy that
        # is merely waiting on the server.
        allowed = kind != "park_arms" and not (
            kind == "run_policy" and self.holder.client_spoke
        )
        for side, values in self.holder.tick(self._now(), allowed):
            message = Float64MultiArray()
            message.data = [float(v) for v in values]
            self._hold_pubs[side].publish(message)

    def _step_move(self) -> None:
        """Publish this tick's twist, or nothing once the move is over.

        Nothing here publishes a stop. The chassis has no watchdog, so the stop
        is base_adapter's job: the gate shuts when the step ends and the adapter
        zeroes the wheels at its own rate, fail-closed on a 0.5 s heartbeat.
        That holds even if this node dies mid-move, which a stop published from
        here would not.
        """
        twist = self.mover.step(self._now())
        if twist is None:
            return
        message = Twist()
        message.linear.x, message.linear.y = twist
        self._base_cmd_pub.publish(message)

    # -- loop 2: supervision -------------------------------------------------

    def _supervisor_tick(self) -> None:
        """Ask whether the current step is done, and switch if it is.

        Nothing in here commands an actuator. It starts and stops steps, and the
        starting and stopping is where every actuator change happens -- which is
        why this loop and the execution loop can run at different rates without
        coordinating: they touch different things.

        Both run on the same single-threaded executor, so they never interleave
        and nothing here needs a lock. That is deliberate. Two rates, one thread.
        """
        self._publish_status()

        if self.runner.terminal:
            self._finish_mission()
            return
        if self.runner.held:
            return

        action = self.runner.pending()
        if action is None:
            return
        if self._current is None or self._current is not action:
            self._start_action(action)
            return

        outcome = self._check_action(action)
        if outcome is None:
            return
        ok, reason = outcome
        self._end_action(action, ok, reason)

    def _publish_gate(self) -> None:
        """The base may move only while a navigate step is actually in flight."""
        open_gate = (
            self.runner.phase is Phase.NAVIGATE
            and not self.runner.held
            and not self._estop
            and self._current is not None
            and self._current.kind in ("navigate", "move_base")
        )
        message = Bool()
        message.data = bool(open_gate)
        self._base_enable_pub.publish(message)

    def _publish_status(self) -> None:
        status: Dict[str, object] = self.runner.status()
        status["estop"] = self._estop
        status["joint_states"] = self._state_seen
        status["mock_policy"] = self.mock_policy
        status["mock_nav"] = self.mock_nav
        if self._current is not None and self._current.kind == "navigate":
            status["distance_remaining"] = self.nav.distance_remaining
        if self._axis_cm is not None:
            status["axis_cm"] = round(self._axis_cm, 1)
        if self._current is not None and self._current.kind == "move_base":
            status["move_remaining_cm"] = round(self.mover.remaining_m * 100.0, 1)
        if self.policies.running:
            status["policy_elapsed_s"] = round(self.policies.alive_for(), 1)
        message = String()
        message.data = json.dumps(status)
        self._status_pub.publish(message)

    # -- per-step handling ---------------------------------------------------

    def _start_action(self, action: Action) -> None:
        self.get_logger().info(f"--> {action.describe()}")
        self._current = action
        self._monitor = None
        self._phase_started = self._now()

        if action.kind == "navigate":
            assert action.station is not None
            if self.mock_nav:
                self.get_logger().warning(
                    f"MOCK: not sending a Nav2 goal to {action.station.name}"
                )
                return
            self.nav.send(action.station, self.mission.frame_id, self.nav_timeout_s)
            return

        if action.kind == "move_base":
            assert action.position is not None
            if self._axis_cm is None:
                self._end_action(action, False, "the mission never declared start_position")
                return
            distance_cm = action.position.axis_cm - self._axis_cm
            distance_m = distance_cm / 100.0
            self.get_logger().info(
                f"    move {distance_cm:+.1f} cm on the {action.step.axis} axis "
                f"({self._axis_cm:+.1f} -> {action.position.axis_cm:+.1f})"
            )
            if self.mock_nav:
                self.get_logger().warning(
                    f"MOCK: not driving the base to {action.position.name}"
                )
                self._axis_cm = action.position.axis_cm
                return
            if not self._wheel_rpm_seen:
                self.get_logger().warning(
                    "no wheel feedback seen yet -- the move will abort in "
                    "2 s if the chassis driver is not up"
                )
            error = self.mover.start(distance_m, action.step.axis, self._now())
            if error is not None:
                self._end_action(action, False, error)
            return

        if action.kind == "run_policy":
            assert action.policy is not None and action.until is not None
            try:
                self.policies.start(action.policy, action.run_label)
            except PolicyPrepareError as exc:
                # The controller was not re-initialised, so no client started and
                # nothing has moved. Fail the phase here rather than monitoring
                # one that never began.
                self._end_action(action, False, str(exc))
                return
            self._monitor = PhaseMonitor(
                action.until,
                self.monitor_config,
                self._now(),
                envelope=(
                    self.envelopes.get(action.until.envelope)
                    if action.until.envelope
                    else None
                ),
            )
            self._route_signal(action.until.signal)
            # The previous client is gone and this one has not published yet, so
            # the hold may cover the startup gap until its first chunk lands.
            self.holder.begin_phase()
            return

        if action.kind == "park_arms":
            assert action.profile is not None
            error = self.parker.start(action.profile)
            if error is not None:
                self._end_action(action, False, error)
            return

        # check_arms and wait: nothing to start.

    def _route_signal(self, name: Optional[str]) -> None:
        """Point this phase's signal at its monitor, and unhook every other.

        Routing rather than always-listening is what stops a reading that
        arrives between phases -- a scanner firing while the robot is parking --
        from being latched into a phase that has not started yet.
        """
        for source_name, source in self.signals.items():
            source.route_to(None)
        if name is None or self._monitor is None:
            return
        source = self.signals.get(name)
        if source is None:
            return
        monitor = self._monitor
        if source.signal.mode == "event" or not source.signal.arm:
            monitor.arm_signal_now()
        source.route_to(monitor.observe_signal)

    def _check_action(self, action: Action) -> Optional[Tuple[bool, str]]:
        """Is this step finished? None means carry on. Commands nothing."""
        elapsed = self._now() - self._phase_started

        if action.kind == "navigate":
            if self.mock_nav:
                return (True, "navigation mocked") if elapsed > 1.0 else None
            return self.nav.poll()

        if action.kind == "move_base":
            if self.mock_nav:
                return (True, "base move mocked") if elapsed > 1.0 else None
            if elapsed > self.move_timeout_s:
                return (
                    False,
                    f"base move timed out after {self.move_timeout_s:.0f}s, "
                    f"{self.mover.remaining_m * 100.0:.1f} cm short",
                )
            return self.mover.outcome(self._now())

        if action.kind == "run_policy":
            died = self.policies.check()
            if died is not None:
                return died
            assert self._monitor is not None
            # The monitor was fed by the joint-state subscription as the samples
            # arrived; this only asks it for an answer.
            verdict = self._monitor.verdict(self._now())
            return (verdict.ok, verdict.reason) if verdict.done else None

        if action.kind == "park_arms":
            if elapsed > self.park_timeout_s:
                return (False, f"park timed out after {self.park_timeout_s:.0f}s")
            return self.parker.outcome()

        if action.kind == "check_arms":
            return self._check_arms(action, elapsed)

        if elapsed >= float(action.seconds or 0.0):
            return (True, f"waited {elapsed:.1f}s")
        return None

    def _check_arms(self, action: Action, elapsed: float) -> Optional[Tuple[bool, str]]:
        """Wait for the arms to be inside the envelope; fail if they never are.

        This is polled rather than sampled once because a policy phase can end
        the instant its condition holds, with the last commanded pose still
        being tracked -- a few hundred milliseconds of settling is normal and is
        not a reason to abort a mission.
        """
        envelope = self.envelopes[str(action.step.envelope)]
        deadline = float(action.seconds or 5.0)
        if self._state_seen and self._positions is not None:
            violation = envelope.check(self._positions)
            if violation is None:
                return (True, f"arms inside envelope {envelope.name!r} after {elapsed:.1f}s")
            if elapsed >= deadline:
                return (False, violation)
            return None
        if elapsed >= deadline:
            return (False, f"no /joint_states -- cannot verify envelope {envelope.name!r}")
        return None

    def _end_action(self, action: Action, ok: bool, reason: str) -> None:
        if action.kind == "run_policy":
            self.policies.stop(f"phase over: {reason}")
            self._switch_began = self._now()
        elif action.kind == "move_base":
            # Book the distance actually achieved, whether the move succeeded or
            # not: a failed move still moved the robot, and the next step's
            # distance has to be computed from where it really is.
            if self._axis_cm is not None and not self.mock_nav:
                self._axis_cm += self.mover.travelled_m * 100.0
            self.mover.cancel()
        elif action.kind == "navigate" and not ok:
            self.nav.cancel("step failed")
        elif action.kind == "park_arms" and not ok:
            self.parker.cancel()
        self._current = None
        self._monitor = None
        self._route_signal(None)
        # Two call sites on purpose. rclpy caches a logger's severity against
        # the (file, function, line) it was called from, so a single line used
        # for both info and error raises "Logger severity cannot be changed
        # between calls" the first time a step fails -- which took down the node
        # on the first failing step of a mission that had worked until then.
        if ok:
            self.get_logger().info(f"<-- ok: {reason}")
        else:
            self.get_logger().error(f"<-- FAILED: {reason}")
        self.runner.report(ok, reason)

    def _stop_activity(self, why: str) -> None:
        """Bring every actuator this node owns to rest, now."""
        if self.policies.running:
            self.policies.stop(why)
        self.nav.cancel(why)
        self.mover.cancel()
        self.parker.cancel()
        self._current = None
        self._monitor = None

    def _finish_mission(self) -> None:
        if self._summary_printed:
            return
        self._summary_printed = True
        self._stop_activity("mission over")
        summary = self.runner.summary()
        if self.runner.phase is Phase.DONE:
            self.get_logger().info("\n" + summary)
        else:
            self.get_logger().error("\n" + summary)
        if self.shutdown_when_done:
            self.get_logger().info("shutting down (shutdown_when_done)")
            raise SystemExit(0 if self.runner.phase is Phase.DONE else 1)

    # -- services ------------------------------------------------------------

    def _srv_start(self, _request, response):
        if self.runner.terminal:
            response.success, response.message = False, f"mission is {self.runner.phase.value}"
            return response
        if self.runner._started:  # noqa: SLF001 -- the runner owns this flag
            response.success, response.message = False, "already started"
            return response
        self._begin_mission("start service")
        response.success = not self.runner.terminal
        response.message = self.runner.reason or "started"
        return response

    def _srv_hold(self, _request, response):
        self._stop_activity("hold service")
        self.runner.hold("hold service")
        response.success, response.message = True, "held; the current step restarts on resume"
        return response

    def _srv_resume(self, _request, response):
        if self._estop:
            response.success, response.message = False, "e-stop is still asserted"
            return response
        if not self.runner.held:
            response.success, response.message = False, f"not held ({self.runner.phase.value})"
            return response
        self.runner.resume()
        response.success, response.message = True, "resumed"
        return response

    def _srv_abort(self, _request, response):
        self._stop_activity("abort service")
        self.runner.abort("abort service")
        response.success, response.message = True, "aborted"
        return response

    def _srv_advance(self, _request, response):
        if self._monitor is None:
            response.success, response.message = False, "no policy phase is running"
            return response
        self._monitor.operator_advance()
        response.success, response.message = True, "phase will finish as a success"
        return response

    def _srv_skip(self, _request, response):
        if self._monitor is None:
            response.success, response.message = False, "no policy phase is running"
            return response
        self._monitor.operator_skip()
        response.success, response.message = True, "phase will finish as a failure"
        return response

    # -- helpers -------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def shutdown(self) -> None:
        self._stop_activity("node shutdown")
        message = Bool()
        message.data = False
        try:
            self._base_enable_pub.publish(message)
        except Exception:  # pragma: no cover -- context may already be gone
            pass


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[OrchestratorNode] = None
    code = 0
    try:
        node = OrchestratorNode()
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code or 0)
    except KeyboardInterrupt:
        code = 130
    except MissionError as exc:
        print(f"mission error: {exc}")
        code = 2
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
