"""The mission file: stations, policies, and the steps that visit them.

A mission is data, not code. It names the base poses the robot drives to, the
GR00T policy sessions it runs there, and the order of both. Nothing in here
imports ROS, so a mission can be validated on a laptop:

    python -m cerebel_orchestrator.mission missions/two_station_pick_place.yaml

Validation is deliberately strict and loud. A mission that names a station or a
policy that does not exist, or a policy phase with no timeout, is a mission that
would fail on hardware with the arms already in the air -- so it fails here
instead.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml

# Every policy phase must carry a hard cap. A GR00T policy has no notion of
# "done" -- it will happily keep predicting actions forever -- so the only thing
# that guarantees a phase ends is the clock.
MAX_TIMEOUT_S = 600.0

ON_FAIL_CHOICES = ("abort", "retry", "continue")
STEP_KINDS = ("navigate", "run_policy", "park_arms", "check_arms", "wait")
GRASP_STATES = ("closed", "open")
SIDES = ("left", "right")


class MissionError(ValueError):
    """A mission file that cannot be executed as written."""


def _require(cond: bool, where: str, msg: str) -> None:
    if not cond:
        raise MissionError(f"{where}: {msg}")


def _as_float(value: Any, where: str, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        raise MissionError(f"{where}: {name} must be a number, got {value!r}") from None


@dataclass(frozen=True)
class Station:
    """A base pose the robot navigates to, in the mission's frame."""

    name: str
    x: float
    y: float
    yaw: float

    @staticmethod
    def parse(name: str, raw: Any) -> "Station":
        where = f"stations.{name}"
        _require(isinstance(raw, dict), where, "must be a mapping with x, y, yaw")
        unknown = set(raw) - {"x", "y", "yaw"}
        _require(not unknown, where, f"unknown keys {sorted(unknown)}")
        for key in ("x", "y"):
            _require(key in raw, where, f"missing {key}")
        yaw = _as_float(raw.get("yaw", 0.0), where, "yaw")
        _require(
            -2 * math.pi - 1e-9 <= yaw <= 2 * math.pi + 1e-9,
            where,
            f"yaw {yaw} is out of range -- it is radians, not degrees",
        )
        return Station(
            name=name,
            x=_as_float(raw["x"], where, "x"),
            y=_as_float(raw["y"], where, "y"),
            yaw=yaw,
        )


@dataclass(frozen=True)
class Policy:
    """One GR00T policy session.

    ``task_description`` must match the training annotation verbatim -- it is the
    language input the policy was fine-tuned against, not a label.

    ``server_host``/``server_port`` are how a *different checkpoint* is
    selected: a policy server serves exactly one checkpoint, so two checkpoints
    in one mission means two servers on two ports. See docs/POLICY_SWITCHING.md.

    ``params`` is passed straight through to the inference client as ROS
    parameters, so anything in adibot_gr00t_client's ARGUMENTS.md is settable
    per phase -- execution_horizon, prefetch_enable, rtc_enable, enable_limits,
    limits_file, and the topic names.
    """

    name: str
    task_description: str
    server_host: str
    server_port: int
    checkpoint_label: str = ""
    params: Dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def parse(name: str, raw: Any) -> "Policy":
        where = f"policies.{name}"
        _require(isinstance(raw, dict), where, "must be a mapping")
        unknown = set(raw) - {
            "task_description",
            "server_host",
            "server_port",
            "checkpoint_label",
            "params",
        }
        _require(not unknown, where, f"unknown keys {sorted(unknown)}")
        _require("task_description" in raw, where, "missing task_description")
        task = raw["task_description"]
        _require(
            isinstance(task, str) and task.strip() != "",
            where,
            "task_description must be a non-empty string matching the training annotation",
        )
        _require("server_host" in raw, where, "missing server_host")
        _require("server_port" in raw, where, "missing server_port")
        try:
            port = int(raw["server_port"])
        except (TypeError, ValueError):
            raise MissionError(f"{where}: server_port must be an integer") from None
        _require(0 < port < 65536, where, f"server_port {port} out of range")
        params = raw.get("params", {}) or {}
        _require(isinstance(params, dict), f"{where}.params", "must be a mapping")
        # task_description and the server address are set by the orchestrator
        # from the fields above; letting params override them would make the
        # mission lie about what ran.
        for reserved in ("task_description", "server_host", "server_port", "checkpoint_label"):
            _require(
                reserved not in params,
                f"{where}.params",
                f"{reserved} belongs in the policy body, not in params",
            )
        return Policy(
            name=name,
            task_description=task,
            server_host=str(raw["server_host"]),
            server_port=port,
            checkpoint_label=str(raw.get("checkpoint_label", "")),
            params=dict(params),
        )


@dataclass(frozen=True)
class Until:
    """When a policy phase is finished.

    ``timeout_s`` is mandatory and is the *only* condition that always fires.

    **Conditions combine with AND.** A phase with both ``grasp`` and ``settled``
    ends when the gripper is closed on the object *and* the arm has stopped
    moving -- which is exactly the question "has the policy finished the pick and
    come to rest in its carry pose", and it is the check that has to pass before
    the base drives off with the object in hand.

    grasp
        ``closed`` or ``open`` on ``side``'s finger joint, held for ``hold_s``.
        The thresholds are not here -- they are a property of the gripper, so
        they live in the orchestrator params file.
    settled
        no *arm* joint moves more than the motion threshold for ``hold_s``. The
        fingers are excluded: a gripper still closing is not an arm still moving.
    operator
        no automatic end at all -- the phase runs until the ``advance`` or
        ``skip`` service is called. Cannot be combined with ``grasp`` or
        ``settled``, because "wait for a human" and "watch for a condition" are
        different intentions and a mixture of the two reads as neither.

    ``advance`` and ``skip`` work in *any* policy phase as an operator override;
    ``operator: true`` declares that they are the only way this one ends.
    """

    timeout_s: float
    grasp: Optional[str] = None
    side: Optional[str] = None
    settled: bool = False
    operator: bool = False
    hold_s: float = 0.5

    @staticmethod
    def parse(raw: Any, where: str) -> "Until":
        _require(isinstance(raw, dict), where, "must be a mapping with at least timeout_s")
        unknown = set(raw) - {"timeout_s", "grasp", "side", "settled", "operator", "hold_s"}
        _require(not unknown, where, f"unknown keys {sorted(unknown)}")
        _require(
            "timeout_s" in raw,
            where,
            "missing timeout_s -- a policy phase with no hard cap never ends",
        )
        timeout = _as_float(raw["timeout_s"], where, "timeout_s")
        _require(0 < timeout <= MAX_TIMEOUT_S, where, f"timeout_s must be in (0, {MAX_TIMEOUT_S}]")
        grasp = raw.get("grasp")
        side = raw.get("side")
        if grasp is not None:
            _require(grasp in GRASP_STATES, where, f"grasp must be one of {GRASP_STATES}")
            _require(side is not None, where, "grasp needs side: left or right")
        if side is not None:
            _require(side in SIDES, where, f"side must be one of {SIDES}")
        hold = _as_float(raw.get("hold_s", 0.5), where, "hold_s")
        _require(
            0 <= hold < timeout,
            where,
            "hold_s must be non-negative and shorter than timeout_s",
        )
        operator = bool(raw.get("operator", False))
        _require(
            not (operator and (grasp is not None or bool(raw.get("settled", False)))),
            where,
            "operator cannot be combined with grasp or settled -- operator means "
            "this phase has no automatic end",
        )
        return Until(
            timeout_s=timeout,
            grasp=grasp,
            side=side,
            settled=bool(raw.get("settled", False)),
            operator=operator,
            hold_s=hold,
        )

    @property
    def timeout_is_success(self) -> bool:
        """True when the timeout is the *intended* end of the phase.

        With no early condition set, running the policy for ``timeout_s`` is
        exactly what the step asked for, so hitting the clock is not a failure.
        """
        return self.grasp is None and not self.settled and not self.operator


@dataclass(frozen=True)
class Step:
    """One entry in the mission's step list."""

    index: int
    kind: str
    station: Optional[str] = None
    policy: Optional[str] = None
    profile: Optional[str] = None
    envelope: Optional[str] = None
    seconds: Optional[float] = None
    until: Optional[Until] = None
    on_fail: str = "abort"
    retries: int = 0
    # run_policy only: this policy is trained to finish in a pose the base can
    # drive with -- holding the object, arms clear of the chassis. It is a claim
    # about the checkpoint, so the lint believes it and check_arms verifies it.
    ends_parked: bool = False

    def describe(self) -> str:
        if self.kind == "navigate":
            return f"navigate -> {self.station}"
        if self.kind == "run_policy":
            tail = " (ends parked)" if self.ends_parked else ""
            return f"run_policy {self.policy}{tail}"
        if self.kind == "park_arms":
            return f"park_arms [{self.profile}]"
        if self.kind == "check_arms":
            return f"check_arms [{self.envelope}]"
        return f"wait {self.seconds}s"

    @staticmethod
    def parse(index: int, raw: Any) -> "Step":
        where = f"steps[{index}]"
        _require(isinstance(raw, dict), where, "must be a mapping")
        kind = raw.get("step")
        _require(kind in STEP_KINDS, where, f"step must be one of {STEP_KINDS}, got {kind!r}")
        allowed = {"step", "on_fail", "retries"} | {
            "navigate": {"station"},
            "run_policy": {"policy", "until", "ends_parked"},
            "park_arms": {"profile"},
            "check_arms": {"envelope", "seconds"},
            "wait": {"seconds"},
        }[kind]
        unknown = set(raw) - allowed
        _require(not unknown, where, f"unknown keys {sorted(unknown)} for step {kind}")

        on_fail = str(raw.get("on_fail", "abort"))
        _require(on_fail in ON_FAIL_CHOICES, where, f"on_fail must be one of {ON_FAIL_CHOICES}")
        retries = raw.get("retries", 0)
        _require(
            isinstance(retries, int) and not isinstance(retries, bool) and retries >= 0,
            where,
            "retries must be a non-negative integer",
        )
        if retries and on_fail != "retry":
            raise MissionError(f"{where}: retries set but on_fail is {on_fail!r}, not 'retry'")
        if on_fail == "retry":
            _require(retries > 0, where, "on_fail: retry needs retries > 0")

        station = policy = profile = envelope = seconds = until = None
        ends_parked = False
        if kind == "navigate":
            station = raw.get("station")
            _require(isinstance(station, str) and station, where, "navigate needs station")
        elif kind == "run_policy":
            policy = raw.get("policy")
            _require(isinstance(policy, str) and policy, where, "run_policy needs policy")
            until = Until.parse(raw.get("until"), f"{where}.until")
            ends_parked = bool(raw.get("ends_parked", False))
            if ends_parked and not (until.settled or until.operator):
                raise MissionError(
                    f"{where}: ends_parked claims the policy finishes in a "
                    "drive-safe pose, but until has no `settled` -- without it "
                    "the phase can end mid-motion and the claim means nothing"
                )
        elif kind == "park_arms":
            profile = str(raw.get("profile", "travel"))
        elif kind == "check_arms":
            envelope = str(raw.get("envelope", ""))
            _require(bool(envelope), where, "check_arms needs an envelope name")
            seconds = _as_float(raw.get("seconds", 5.0), where, "seconds")
            _require(0 < seconds <= MAX_TIMEOUT_S, where, "seconds out of range")
        else:
            seconds = _as_float(raw.get("seconds", 1.0), where, "seconds")
            _require(0 < seconds <= MAX_TIMEOUT_S, where, "seconds out of range")

        return Step(
            index=index,
            kind=kind,
            station=station,
            policy=policy,
            profile=profile,
            envelope=envelope,
            seconds=seconds,
            until=until,
            on_fail=on_fail,
            retries=retries,
            ends_parked=ends_parked,
        )


@dataclass(frozen=True)
class Mission:
    name: str
    frame_id: str
    stations: Dict[str, Station]
    policies: Dict[str, Policy]
    steps: List[Step]
    repeat: int = 1

    @staticmethod
    def from_dict(raw: Any) -> "Mission":
        _require(isinstance(raw, dict), "mission", "the file must contain a mapping")
        unknown = set(raw) - {"name", "frame_id", "stations", "policies", "steps", "repeat"}
        _require(not unknown, "mission", f"unknown top-level keys {sorted(unknown)}")
        _require("steps" in raw, "mission", "missing steps")

        stations = {
            name: Station.parse(name, body)
            for name, body in (raw.get("stations") or {}).items()
        }
        policies = {
            name: Policy.parse(name, body)
            for name, body in (raw.get("policies") or {}).items()
        }
        steps_raw = raw["steps"]
        _require(isinstance(steps_raw, list) and steps_raw, "steps", "must be a non-empty list")
        steps = [Step.parse(i, body) for i, body in enumerate(steps_raw)]

        repeat = raw.get("repeat", 1)
        _require(
            isinstance(repeat, int) and not isinstance(repeat, bool) and repeat >= 1,
            "mission.repeat",
            "must be an integer >= 1",
        )

        # Cross-references last, so the message names the step, not the table.
        for step in steps:
            if step.kind == "navigate" and step.station not in stations:
                raise MissionError(
                    f"steps[{step.index}]: station {step.station!r} is not in stations "
                    f"({sorted(stations) or 'none defined'})"
                )
            if step.kind == "run_policy" and step.policy not in policies:
                raise MissionError(
                    f"steps[{step.index}]: policy {step.policy!r} is not in policies "
                    f"({sorted(policies) or 'none defined'})"
                )

        return Mission(
            name=str(raw.get("name", "unnamed")),
            frame_id=str(raw.get("frame_id", "odom")),
            stations=stations,
            policies=policies,
            steps=steps,
            repeat=repeat,
        )

    @staticmethod
    def load(path: str) -> "Mission":
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        try:
            return Mission.from_dict(raw)
        except MissionError as exc:
            raise MissionError(f"{path}: {exc}") from None

    def policy_ports(self) -> Dict[str, List[str]]:
        """Which policies share a host:port -- i.e. share one checkpoint.

        Two policies on the same address are two prompts to the *same* served
        checkpoint. That is legitimate, and it is also the most common way a
        mission quietly does the wrong thing, so the orchestrator prints this
        grouping at startup.
        """
        grouped: Dict[str, List[str]] = {}
        for policy in self.policies.values():
            grouped.setdefault(f"{policy.server_host}:{policy.server_port}", []).append(policy.name)
        return grouped


def navigate_safety_warnings(mission: Mission) -> List[str]:
    """Things that are legal but probably wrong. Warnings, never errors.

    The one that matters: driving the base with the arms wherever a policy left
    them. The arms hold their last commanded pose after the inference client
    exits, so a navigate step moves the robot with two 7-DOF arms sticking out at
    whatever the last action chunk asked for.

    Three things make a navigate safe, and any of them satisfies the check:

    * ``park_arms`` -- the orchestrator put the arms somewhere known;
    * ``check_arms`` -- the arms were verified to be inside a named envelope;
    * a ``run_policy`` marked ``ends_parked: true`` -- the policy itself finishes
      in a drive-safe pose. This is the pick-and-carry case: the policy picks the
      object and holds it in its carry pose, so parking in between would drop it.

    These are warnings because a short, well-understood move is a legitimate
    reason to skip all three, and the orchestrator should not refuse to run it.
    """
    warnings: List[str] = []
    steps = mission.steps
    if not any(step.kind == "navigate" for step in steps):
        return warnings
    settles_the_arms = [
        step
        for step in steps
        if step.kind in ("park_arms", "check_arms")
        or (step.kind == "run_policy" and step.ends_parked)
    ]
    if not settles_the_arms:
        warnings.append(
            "the mission navigates but never parks, checks or declares the arms "
            "safe -- the base will move with them wherever the policy left them"
        )
        return warnings

    # Walk backwards from each navigate to the last step that moved the arms.
    # Only run_policy and park_arms do; navigate, wait and check_arms leave them
    # alone, so a park several steps back is still a park.
    order = list(range(len(steps)))
    if mission.repeat > 1:
        # Cycles 2+ arrive at step 0 from the last step of the previous cycle.
        order = order + order

    for position in range(len(order)):
        step = steps[order[position]]
        if step.kind != "navigate":
            continue
        culprit: Optional[Step] = None
        for back in range(position - 1, -1, -1):
            previous = steps[order[back]]
            if previous.kind in ("park_arms", "check_arms"):
                break
            if previous.kind == "run_policy":
                # A policy that ends in its carry pose is the point of the
                # pick-and-carry mission, not a mistake.
                if not previous.ends_parked:
                    culprit = previous
                break
        else:
            # Ran off the front of the list without meeting either.
            if position < len(steps):
                warnings.append(
                    f"steps[{step.index}] navigates before any park_arms or "
                    "check_arms, so the mission assumes the arms are already "
                    "parked when it starts"
                )
            continue
        if culprit is not None:
            warnings.append(
                f"steps[{step.index}] ({step.describe()}) follows "
                f"{culprit.describe()} with no park_arms or check_arms in "
                "between, and that policy is not marked ends_parked"
            )
    # The doubled list can report the same step twice; keep the first of each.
    seen: Dict[str, None] = {}
    for warning in warnings:
        seen.setdefault(warning, None)
    return list(seen)


def main(argv: Optional[List[str]] = None) -> int:
    """Validate the mission files named on the command line."""
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        print("usage: mission_check MISSION.yaml [MISSION.yaml ...]", file=sys.stderr)
        return 2
    failed = 0
    for path in args:
        try:
            mission = Mission.load(path)
        except (MissionError, OSError, yaml.YAMLError) as exc:
            print(f"FAIL {path}\n     {exc}")
            failed += 1
            continue
        print(f"OK   {path}")
        print(f"     name={mission.name} frame={mission.frame_id} repeat={mission.repeat}")
        print(f"     stations: {', '.join(sorted(mission.stations)) or 'none'}")
        for address, names in sorted(mission.policy_ports().items()):
            shared = "  <-- one checkpoint, several prompts" if len(names) > 1 else ""
            print(f"     {address}: {', '.join(sorted(names))}{shared}")
        for step in mission.steps:
            print(f"     [{step.index}] {step.describe()}")
        for warning in navigate_safety_warnings(mission):
            print(f"     WARN {warning}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
