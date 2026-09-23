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
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .arm_park import ArmParker, load_park_poses
from .envelopes import load_envelopes
from .joints import reorder_by_name
from .mission import Mission, MissionError, navigate_safety_warnings
from .nav_client import NavClient
from .phase_monitor import MonitorConfig, PhaseMonitor
from .policy_runner import PolicyRunner, RunnerConfig, describe_command
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
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("nav_timeout_s", 120.0)
        self.declare_parameter("nav_server_wait_s", 20.0)

        # Policies
        self.declare_parameter(
            "policy_cmd", ["ros2", "run", "adibot_gr00t_client", "inference_client"]
        )
        self.declare_parameter("log_dir", "~/adibot_logs")
        self.declare_parameter("policy_startup_grace_s", 20.0)

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

        self.nav = NavClient(self, str(get("nav_action").value))
        self.policies = PolicyRunner(
            RunnerConfig(
                policy_cmd=[str(item) for item in get("policy_cmd").value],
                log_dir=str(get("log_dir").value),
                mock=self.mock_policy,
                startup_grace_s=self.policy_startup_grace_s,
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

        self._positions: Optional[List[float]] = None
        self._state_seen = False
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, JOINT_STATE_QOS
        )
        self.create_subscription(Bool, str(get("estop_topic").value), self._on_estop, 10)
        self._base_enable_pub = self.create_publisher(
            Bool, str(get("base_enable_topic").value), 10
        )
        self._status_pub = self.create_publisher(String, str(get("status_topic").value), 10)

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
        missing = sorted(name for name in wanted if name not in self.envelopes)
        if missing:
            raise MissionError(
                f"check_arms names envelope(s) {missing} that are not in "
                f"arm_envelopes_file (have {sorted(self.envelopes) or 'none'})"
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
        if not self._nav_ready and self._needs_nav():
            wait = float(self.get_parameter("nav_server_wait_s").value)
            self._nav_ready = self.nav.server_ready(wait)
            if not self._nav_ready:
                self.runner.fault(
                    f"Nav2 action server {self.nav.action_name!r} did not appear "
                    f"within {wait:.0f}s -- is the navigation stack up?"
                )
                return
        self.get_logger().info(f"mission start ({why})")
        self.runner.start()

    def _needs_nav(self) -> bool:
        if self.mock_nav:
            return False
        return any(step.kind == "navigate" for step in self.mission.steps)

    # -- subscriptions -------------------------------------------------------

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
        if (
            self._current is not None
            and self._current.kind == "park_arms"
            and not self.runner.held
            and not self._estop
        ):
            self.parker.step()

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
            and self._current.kind == "navigate"
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

        if action.kind == "run_policy":
            assert action.policy is not None and action.until is not None
            self.policies.start(action.policy, action.run_label)
            self._monitor = PhaseMonitor(action.until, self.monitor_config, self._now())
            return

        if action.kind == "park_arms":
            assert action.profile is not None
            error = self.parker.start(action.profile)
            if error is not None:
                self._end_action(action, False, error)
            return

        # check_arms and wait: nothing to start.

    def _check_action(self, action: Action) -> Optional[Tuple[bool, str]]:
        """Is this step finished? None means carry on. Commands nothing."""
        elapsed = self._now() - self._phase_started

        if action.kind == "navigate":
            if self.mock_nav:
                return (True, "navigation mocked") if elapsed > 1.0 else None
            return self.nav.poll()

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
        elif action.kind == "navigate" and not ok:
            self.nav.cancel("step failed")
        elif action.kind == "park_arms" and not ok:
            self.parker.cancel()
        self._current = None
        self._monitor = None
        level = self.get_logger().info if ok else self.get_logger().error
        level(f"<-- {'ok' if ok else 'FAILED'}: {reason}")
        self.runner.report(ok, reason)

    def _stop_activity(self, why: str) -> None:
        """Bring every actuator this node owns to rest, now."""
        if self.policies.running:
            self.policies.stop(why)
        self.nav.cancel(why)
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
