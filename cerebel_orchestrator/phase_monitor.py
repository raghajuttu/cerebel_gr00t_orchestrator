"""When a policy phase is finished.

A GR00T policy does not report success. It returns an action chunk for whatever
observation it is given, forever. So "the pick is done" has to be read off the
robot, and this module is where that reading lives -- pure, clock-injected, and
unit-testable against a synthetic joint-state stream.

Three conditions, all of them optional except the first:

``timeout_s``
    always present, always fires. When the step asked for nothing else, the
    timeout is the intended end of the phase and counts as success
    (``Until.timeout_is_success``); otherwise it is a failure.
``grasp: closed|open`` on ``side``
    the finger joint crosses its threshold and stays there for ``hold_s``.
``settled``
    no joint moves more than ``motion_eps`` for ``hold_s``.

Both early conditions have to be *armed* before they can fire, because the state
they look for is often true at the moment the phase starts: the gripper is
already open when a pick begins, and every joint is stationary in the moment
before the first action chunk lands. Arming is deliberately different for the
two:

* ``grasp`` arms on seeing the opposite state -- a pick that must end closed has
  to have been open, so the check cannot pass on the starting pose.
* ``settled`` arms on seeing motion, and additionally never before
  ``settle_grace_s`` has elapsed, so a slow-to-start policy is not mistaken for
  a finished one.

If a condition never arms, the phase runs to its timeout. That is the safe
direction: the robot keeps working on the task rather than being declared done
while it stands still.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

from .joints import GRIPPER_INDEX
from .mission import Until


@dataclass(frozen=True)
class MonitorConfig:
    """Gripper and motion thresholds -- properties of the robot, not the mission.

    The defaults are placeholders. Measure the real numbers off an existing run
    log: open the per-tick CSV from ``adibot_gr00t_client`` in
    ``adibot_run_browser`` and read the finger joint's value while empty, while
    holding the cube, and while released. Put the two thresholds either side of
    the gap, not at the extremes.
    """

    grasp_close_m: float = 0.010
    grasp_open_m: float = 0.030
    motion_eps: float = 0.004
    settle_grace_s: float = 3.0
    stale_state_s: float = 1.0

    def validate(self) -> None:
        if not self.grasp_close_m < self.grasp_open_m:
            raise ValueError(
                f"grasp_close_m ({self.grasp_close_m}) must be below grasp_open_m "
                f"({self.grasp_open_m}); both are finger positions in metres"
            )
        if self.motion_eps <= 0:
            raise ValueError("motion_eps must be positive")
        if self.settle_grace_s < 0 or self.stale_state_s <= 0:
            raise ValueError("settle_grace_s must be >= 0 and stale_state_s > 0")


@dataclass(frozen=True)
class Verdict:
    """The monitor's answer for this tick."""

    done: bool
    ok: bool = False
    reason: str = ""

    @staticmethod
    def running() -> "Verdict":
        return Verdict(done=False)


class PhaseMonitor:
    """Decides when one ``run_policy`` step is over.

    ``now`` is supplied by the caller on every call -- the orchestrator passes
    the ROS clock, the tests pass integers.
    """

    def __init__(self, until: Until, config: MonitorConfig, start_time: float) -> None:
        config.validate()
        self.until = until
        self.config = config
        self.start_time = start_time
        self._last_positions: Optional[List[float]] = None
        self._last_state_time: Optional[float] = None
        self._grasp_armed = False
        self._grasp_since: Optional[float] = None
        self._motion_seen = False
        self._still_since: Optional[float] = None
        self._operator_advanced = False
        self._operator_aborted = False

    # -- external pokes -----------------------------------------------------

    def operator_advance(self) -> None:
        """The ``advance`` service was called: finish this phase successfully."""
        self._operator_advanced = True

    def operator_skip(self) -> None:
        """The ``skip`` service was called: finish this phase as a failure."""
        self._operator_aborted = True

    # -- the tick -----------------------------------------------------------

    def update(self, now: float, positions: Optional[Sequence[float]]) -> Verdict:
        """Advance the monitor by one observation.

        ``positions`` is the 16-DOF canonical joint vector, or None when no
        fresh ``/joint_states`` was available this tick.
        """
        elapsed = now - self.start_time

        if self._operator_aborted:
            return Verdict(True, False, "operator skipped the phase")
        if self._operator_advanced:
            return Verdict(True, True, f"operator advanced after {elapsed:.1f}s")

        if positions is not None:
            if len(positions) != 16:
                raise ValueError(f"expected 16 canonical joints, got {len(positions)}")
            self._observe(now, list(float(v) for v in positions))

        # Losing proprioception mid-phase is a fault, not a slow tick: the
        # policy is still driving the arms off an observation nobody is checking.
        if self._last_state_time is not None:
            age = now - self._last_state_time
            if age > self.config.stale_state_s:
                return Verdict(True, False, f"joint_states stale for {age:.2f}s")

        if self.until.grasp is not None:
            verdict = self._check_grasp(now, elapsed)
            if verdict.done:
                return verdict

        if self.until.settled:
            verdict = self._check_settled(now, elapsed)
            if verdict.done:
                return verdict

        if elapsed >= self.until.timeout_s:
            if self.until.timeout_is_success:
                return Verdict(True, True, f"ran the full {self.until.timeout_s:.0f}s")
            return Verdict(
                True, False, f"timed out after {self.until.timeout_s:.0f}s: {self._unmet()}"
            )

        return Verdict.running()

    # -- internals ----------------------------------------------------------

    def _observe(self, now: float, positions: List[float]) -> None:
        if self._last_positions is not None:
            moved = max(
                abs(new - old) for new, old in zip(positions, self._last_positions)
            )
            if moved > self.config.motion_eps:
                self._motion_seen = True
                self._still_since = None
            elif self._still_since is None:
                self._still_since = now
        self._last_positions = positions
        self._last_state_time = now

    def _finger(self) -> Optional[float]:
        if self._last_positions is None or self.until.side is None:
            return None
        return self._last_positions[GRIPPER_INDEX[self.until.side]]

    def _check_grasp(self, now: float, elapsed: float) -> Verdict:
        finger = self._finger()
        if finger is None:
            return Verdict.running()
        if self.until.grasp == "closed":
            at_target = finger <= self.config.grasp_close_m
            at_opposite = finger >= self.config.grasp_open_m
        else:
            at_target = finger >= self.config.grasp_open_m
            at_opposite = finger <= self.config.grasp_close_m

        if not self._grasp_armed:
            if at_opposite:
                self._grasp_armed = True
            return Verdict.running()

        if not at_target:
            self._grasp_since = None
            return Verdict.running()
        if self._grasp_since is None:
            self._grasp_since = now
        if now - self._grasp_since >= self.until.hold_s:
            return Verdict(
                True,
                True,
                f"{self.until.side} gripper {self.until.grasp} "
                f"({finger:.4f} m) held {self.until.hold_s:.1f}s at {elapsed:.1f}s",
            )
        return Verdict.running()

    def _check_settled(self, now: float, elapsed: float) -> Verdict:
        if not self._motion_seen or elapsed < self.config.settle_grace_s:
            return Verdict.running()
        if self._still_since is None:
            return Verdict.running()
        if now - self._still_since >= self.until.hold_s:
            return Verdict(
                True,
                True,
                f"arms settled for {self.until.hold_s:.1f}s at {elapsed:.1f}s",
            )
        return Verdict.running()

    def _unmet(self) -> str:
        """Why the early conditions did not fire -- the useful half of a timeout."""
        bits: List[str] = []
        if self.until.grasp is not None:
            finger = self._finger()
            where = "no joint_states" if finger is None else f"finger at {finger:.4f} m"
            armed = "armed" if self._grasp_armed else "never armed"
            bits.append(f"grasp {self.until.side} {self.until.grasp} {armed}, {where}")
        if self.until.settled:
            if not self._motion_seen:
                bits.append("settled never armed -- the arms never moved")
            else:
                bits.append("arms never stopped moving")
        if self.until.operator:
            bits.append("no operator advance")
        return "; ".join(bits) or "no early condition set"
