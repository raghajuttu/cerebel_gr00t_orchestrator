"""What to do next -- the mission's control flow, with no ROS and no clock.

``MissionRunner`` is a turn-taking object. The driver asks it for the next
action, carries that action out however it likes, and reports the outcome:

    runner = MissionRunner(mission)
    runner.start()
    while not runner.terminal:
        action = runner.pending()
        if action is None:        # held by the e-stop
            continue
        ok, reason = do_the_thing(action)
        runner.report(ok, reason)

Keeping the clock and the hardware out of here is what makes the interesting
parts -- retries, repeats, what a hold does to a half-finished step -- testable
without a robot.

**A hold restarts the current step.** There is no sane way to resume a policy
phase or a Nav2 goal from the middle after the arms and wheels have been cut,
so the step is re-issued from the beginning on resume, without consuming a
retry. Steps therefore have to be written so that running one twice is safe --
which is true of navigate and park_arms by construction, and is a real
constraint on a run_policy step (see docs/SAFETY.md).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from .mission import Mission, Policy, Position, Station, Step, Until


class Phase(str, Enum):
    """Where the mission is. The string values go straight into the status topic."""

    IDLE = "idle"
    PARK = "park"
    CHECK = "check_arms"
    NAVIGATE = "navigate"
    POLICY = "policy"
    WAIT = "wait"
    HOLD = "hold"
    DONE = "done"
    ABORTED = "aborted"
    FAULT = "fault"


TERMINAL_PHASES = (Phase.DONE, Phase.ABORTED, Phase.FAULT)

_PHASE_FOR_KIND = {
    "navigate": Phase.NAVIGATE,
    # Both kinds drive the base, so both are the same phase. The interlock and
    # the status topic care that the wheels may turn, not which stack turns them.
    "move_base": Phase.NAVIGATE,
    "run_policy": Phase.POLICY,
    "park_arms": Phase.PARK,
    "check_arms": Phase.CHECK,
    "wait": Phase.WAIT,
}


@dataclass(frozen=True)
class Action:
    """One unit of work for the driver, resolved against the mission tables.

    The driver never looks anything up itself: whatever a step referred to by
    name arrives here as the object.
    """

    kind: str
    step: Step
    cycle: int
    attempt: int
    run_label: str
    station: Optional[Station] = None
    position: Optional[Position] = None
    policy: Optional[Policy] = None
    until: Optional[Until] = None
    seconds: Optional[float] = None
    profile: Optional[str] = None
    envelope: Optional[str] = None

    def describe(self) -> str:
        suffix = f" (attempt {self.attempt + 1})" if self.attempt else ""
        return f"cycle {self.cycle} step {self.step.index}: {self.step.describe()}{suffix}"


@dataclass
class Record:
    """One finished attempt, for the run log."""

    cycle: int
    step_index: int
    attempt: int
    kind: str
    detail: str
    ok: bool
    reason: str


class MissionRunner:
    def __init__(self, mission: Mission) -> None:
        self.mission = mission
        self.phase: Phase = Phase.IDLE
        self.cycle: int = 0
        self.step_index: int = 0
        self.attempt: int = 0
        self.reason: str = ""
        self.history: List[Record] = []
        self._in_flight: Optional[Action] = None
        self._phase_before_hold: Optional[Phase] = None
        self._started = False

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        if self._started:
            raise RuntimeError("MissionRunner.start() called twice")
        self._started = True
        self.cycle = 1
        self.step_index = 0
        self.attempt = 0
        self._advance_phase()

    @property
    def terminal(self) -> bool:
        return self.phase in TERMINAL_PHASES

    @property
    def held(self) -> bool:
        return self.phase is Phase.HOLD

    # -- the turn-taking interface -----------------------------------------

    def pending(self) -> Optional[Action]:
        """The action the driver should be carrying out, or None.

        None means there is nothing to do *right now*: the mission is terminal,
        or held, or not started. It is never a signal to move on.
        """
        if self.terminal or self.held or not self._started:
            return None
        if self._in_flight is None:
            self._in_flight = self._build_action()
        return self._in_flight

    def report(self, ok: bool, reason: str = "") -> None:
        """Record the outcome of the pending action and move the mission on."""
        action = self._in_flight
        if action is None:
            raise RuntimeError("report() with no action in flight")
        self._in_flight = None
        self.history.append(
            Record(
                cycle=action.cycle,
                step_index=action.step.index,
                attempt=action.attempt,
                kind=action.kind,
                detail=action.step.describe(),
                ok=ok,
                reason=reason,
            )
        )
        if ok:
            self.attempt = 0
            self._next_step()
            return
        self._handle_failure(action.step, reason)

    # -- interruptions ------------------------------------------------------

    def hold(self, reason: str = "hold asserted") -> None:
        """Freeze the mission. The driver is responsible for stopping motion."""
        if self.terminal or self.held:
            return
        self._phase_before_hold = self.phase
        self._in_flight = None  # the step restarts on resume
        self.phase = Phase.HOLD
        self.reason = reason

    def resume(self) -> None:
        """Re-issue the current step from the beginning."""
        if not self.held:
            return
        self.reason = ""
        self.phase = self._phase_before_hold or Phase.IDLE
        self._phase_before_hold = None
        if self._started and not self.terminal:
            self._advance_phase()

    def abort(self, reason: str) -> None:
        """End the mission now, by operator request or a driver-level failure."""
        if self.terminal:
            return
        self._in_flight = None
        self.phase = Phase.ABORTED
        self.reason = reason

    def fault(self, reason: str) -> None:
        """End the mission because the orchestrator itself is not healthy."""
        if self.terminal:
            return
        self._in_flight = None
        self.phase = Phase.FAULT
        self.reason = reason

    # -- internals ----------------------------------------------------------

    def _current_step(self) -> Step:
        return self.mission.steps[self.step_index]

    def _build_action(self) -> Action:
        step = self._current_step()
        label_bits = [self.mission.name, f"c{self.cycle}", f"s{step.index}", step.kind]
        if step.policy:
            label_bits.append(step.policy)
        if self.attempt:
            label_bits.append(f"a{self.attempt + 1}")
        return Action(
            kind=step.kind,
            step=step,
            cycle=self.cycle,
            attempt=self.attempt,
            run_label="_".join(label_bits),
            station=self.mission.stations.get(step.station) if step.station else None,
            position=self.mission.positions.get(step.position) if step.position else None,
            policy=self.mission.policies.get(step.policy) if step.policy else None,
            until=step.until,
            seconds=step.seconds,
            profile=step.profile,
            envelope=step.envelope,
        )

    def _advance_phase(self) -> None:
        self.phase = _PHASE_FOR_KIND[self._current_step().kind]

    def _next_step(self) -> None:
        self.step_index += 1
        if self.step_index < len(self.mission.steps):
            self._advance_phase()
            return
        if self.cycle < self.mission.repeat:
            self.cycle += 1
            self.step_index = 0
            self._advance_phase()
            return
        self.phase = Phase.DONE
        self.reason = f"{self.mission.repeat} cycle(s) complete"

    def _handle_failure(self, step: Step, reason: str) -> None:
        if step.on_fail == "continue":
            self.attempt = 0
            self._next_step()
            return
        if step.on_fail == "retry" and self.attempt < step.retries:
            self.attempt += 1
            self._advance_phase()
            return
        self.phase = Phase.ABORTED
        if step.on_fail == "retry":
            self.reason = (
                f"step {step.index} ({step.describe()}) failed "
                f"{step.retries + 1}x: {reason}"
            )
        else:
            self.reason = f"step {step.index} ({step.describe()}) failed: {reason}"

    # -- reporting ----------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        """A flat snapshot, cheap enough to publish every tick."""
        step = self._current_step() if self._started and not self.terminal else None
        return {
            "mission": self.mission.name,
            "phase": self.phase.value,
            "cycle": self.cycle,
            "cycles_total": self.mission.repeat,
            "step_index": self.step_index if step else -1,
            "steps_total": len(self.mission.steps),
            "step": step.describe() if step else "",
            "attempt": self.attempt,
            "reason": self.reason,
            "completed": sum(1 for record in self.history if record.ok),
            "failed": sum(1 for record in self.history if not record.ok),
        }

    def summary(self) -> str:
        lines = [f"mission {self.mission.name}: {self.phase.value}"]
        if self.reason:
            lines.append(f"  reason: {self.reason}")
        for record in self.history:
            mark = "ok  " if record.ok else "FAIL"
            attempt = f" (attempt {record.attempt + 1})" if record.attempt else ""
            tail = f" -- {record.reason}" if record.reason else ""
            lines.append(
                f"  {mark} c{record.cycle} s{record.step_index} {record.detail}{attempt}{tail}"
            )
        return "\n".join(lines)
