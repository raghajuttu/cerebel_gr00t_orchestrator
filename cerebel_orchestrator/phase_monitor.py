"""When a policy phase is finished.

A GR00T policy does not report success. It returns an action chunk for whatever
observation it is given, forever. So "the pick is done" has to be read off the
robot, and this module is where that reading lives -- pure, clock-injected, and
unit-testable against a synthetic joint-state stream.

The conditions, all optional except the first, **combined with AND**:

``timeout_s``
    always present, always fires. When the step asked for nothing else, the
    timeout is the intended end of the phase and counts as success
    (``Until.timeout_is_success``); otherwise it is a failure.
``grasp: closed|open`` on ``side``
    the finger joint crosses its threshold and stays there for ``hold_s``.
``settled``
    no *arm* joint has MOVED more than ``still_spread_rad`` over the last
    ``hold_s`` seconds. A position spread, not a speed -- and that distinction
    is load-bearing.

    This was a speed threshold and it was wrong. Teleoperated demonstrations
    jitter: a parked arm wobbles a hundredth of a radian between frames, which
    at 30 fps reads as 0.3 rad/s, well above any threshold loose enough to be
    useful. So the check fired on a momentary dip during the return motion
    rather than at the stop. Measured against all 150 place episodes, it ended
    the phase a median 1.9 s early -- p95 7.5 s -- in 75 of 88 episodes that
    reach a rest. The arm stopped mid-return on hardware, which is how this was
    found.

    A spread over a window cannot be fooled that way: jitter has a small
    spread however fast it looks. Same measurement, position-based: median
    0.024 rad from the pose the arm actually rests at, against 0.143 before.

    The fingers are excluded either way -- a gripper still closing is not an arm
    still moving, and the two are not even in the same units.
``effort``
    the side's finger joint is pushing with at least this much |effort|, held
    for ``hold_s``. This is the better half of the grasp question: a gripper
    whose entire travel is five centimetres gives a few millimetres of position
    signal between "holding a lipstick" and "closed on air", but the whole grip
    force in effort. Where the driver publishes effort, prefer it.
``envelope``
    every constrained joint is inside a named envelope, held for ``hold_s``.
    This is how a phase that ends at a POSE ends. A pick finishes with the
    object lifted clear of the tote, and neither the gripper nor a motion
    threshold says that -- the gripper closes at 44% of a lipstick episode,
    while the arm is still down in the tote, and the arm pauses mid-lift often
    enough that ``settled`` fires at 70%. The joint angles do say it: replayed
    against all 152 pick episodes, gripper-closed AND inside the carry envelope
    fires in 151 of them, at 88% of the episode.
``signal``
    an external sensor fired -- the barcode scanner returned a code, a beam
    broke, a load cell saw the weight arrive. Everything else here is
    proprioception and can only answer questions about the robot; this is the
    only condition that can answer a question about the world. Fed by
    ``observe_signal`` from whatever subscribes to it.

``grasp`` and ``settled`` together are the pick-and-carry check: the object is
held *and* the arm has come to rest in its carry pose. That conjunction is what
has to be true before the base drives off with the object, which is why the two
combine rather than racing.

Both early conditions have to be *armed* before they can fire, because the state
they look for is often true at the moment the phase starts: the gripper is
already open when a pick begins, and every joint is stationary in the moment
before the first action chunk lands. Arming is deliberately different for the
two:

* ``grasp`` arms on seeing the opposite state -- a pick that must end closed has
  to have been open, so the check cannot pass on the starting pose.
* ``effort`` needs no arming: an idle gripper reads near zero, so the condition
  starts false by itself.
* ``envelope`` needs no arming either -- an envelope taken from where a phase
  ENDS does not contain the pose it starts from, which is the same protection
  arming provides, built into the bounds instead of the logic.
* ``signal`` arms according to its own declaration -- an ``event`` signal starts
  unfired and needs nothing, a ``level`` signal must be seen false first unless
  the mission says otherwise.
* ``settled`` arms on seeing motion, and additionally never before
  ``settle_grace_s`` has elapsed, so a slow-to-start policy is not mistaken for
  a finished one.

If a condition never arms, the phase runs to its timeout. That is the safe
direction: the robot keeps working on the task rather than being declared done
while it stands still.

**Observing and deciding are separate calls.** ``observe`` folds in one
``/joint_states`` sample and is meant to be called from the subscription, at
whatever rate the robot publishes; ``verdict`` answers "is this phase over?" and
is meant to be called from the loop that switches steps, at whatever rate that
runs. Keeping them apart is what lets ``motion_eps`` be a **speed** rather than a
per-tick delta -- a threshold measured in "how far a joint moved between two of
my ticks" silently changes meaning the moment either rate changes, which is the
sort of tuning that appears to work and then does not. ``update`` does both in one
call, for tests and for callers with only one loop.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Sequence, Tuple

from .joints import ARM_SLICE, GRIPPER_INDEX
from .mission import Until

# Which canonical indices count as "the arms moving". The two finger joints are
# not in this list; see the module docstring.
ARM_INDICES = [
    index
    for side in ("left", "right")
    for index in range(ARM_SLICE[side].start, ARM_SLICE[side].stop)
]


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
    # A joint SPEED, rad/s. Used ONLY to arm the settled condition -- "has this
    # arm moved at all yet" -- where a jitter spike answering yes does no harm.
    # It no longer decides stillness; see still_spread_rad.
    motion_eps: float = 0.20
    # How far any arm joint may travel within the hold window and still count as
    # parked, in radians. MEASURED against 150 place episodes: at 0.04 over a
    # 1.0 s window the phase ends a median 0.024 rad from the pose the arm
    # actually rests at, against 0.143 for the speed threshold it replaced.
    still_spread_rad: float = 0.04
    settle_grace_s: float = 3.0
    stale_state_s: float = 1.0
    # The staleness check is suppressed for this long after a phase starts. A
    # policy server warming up can starve the joint-state subscription of a few
    # samples, and that is not a reason to fail a phase that has barely begun.
    stale_grace_s: float = 0.0

    def validate(self) -> None:
        if not self.grasp_close_m < self.grasp_open_m:
            raise ValueError(
                f"grasp_close_m ({self.grasp_close_m}) must be below grasp_open_m "
                f"({self.grasp_open_m}); both are finger positions in metres"
            )
        if self.motion_eps <= 0:
            raise ValueError("motion_eps must be positive")
        if self.still_spread_rad <= 0:
            raise ValueError("still_spread_rad must be positive")
        if self.settle_grace_s < 0 or self.stale_state_s <= 0:
            raise ValueError("settle_grace_s must be >= 0 and stale_state_s > 0")
        if self.stale_grace_s < 0:
            raise ValueError("stale_grace_s must be >= 0")


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

    def __init__(
        self,
        until: Until,
        config: MonitorConfig,
        start_time: float,
        envelope=None,
    ) -> None:
        config.validate()
        self.until = until
        self.config = config
        self.start_time = start_time
        # Resolved by the caller from the envelopes file, because the monitor
        # has no business reading files. None when the step asked for no
        # envelope -- and if the step DID ask and this is still None, the
        # condition can never pass, which the timeout reason says out loud.
        self.envelope = envelope
        self._envelope_since: Optional[float] = None
        # A step may demand a stricter stillness than the robot's default.
        self._still_spread = (
            until.still_spread
            if until.still_spread is not None
            else config.still_spread_rad
        )
        self._last_positions: Optional[List[float]] = None
        self._last_state_time: Optional[float] = None
        # Recent arm poses, for the stillness spread. Trimmed to the hold
        # window, so it stays small whatever rate the robot publishes at.
        self._history: Deque[Tuple[float, List[float]]] = deque()
        self._grasp_armed = False
        self._grasp_since: Optional[float] = None
        self._motion_seen = False
        self._operator_advanced = False
        self._operator_aborted = False
        self._last_efforts: Optional[List[float]] = None
        self._effort_since: Optional[float] = None
        self._signal_armed = False
        self._signal_level = False
        self._signal_fired = False
        self._signal_detail = ""

    # -- external pokes -----------------------------------------------------

    def operator_advance(self) -> None:
        """The ``advance`` service was called: finish this phase successfully."""
        self._operator_advanced = True

    def operator_skip(self) -> None:
        """The ``skip`` service was called: finish this phase as a failure."""
        self._operator_aborted = True

    # -- the tick -----------------------------------------------------------

    def observe(self, now: float, positions: Sequence[float]) -> None:
        """Fold in one ``/joint_states`` sample. Call from the subscription.

        Cheap by design: it updates the motion and grasp state and decides
        nothing, so it can run as often as the robot publishes.
        """
        if len(positions) != 16:
            raise ValueError(f"expected 16 canonical joints, got {len(positions)}")
        self._observe(now, [float(value) for value in positions])

    def observe_effort(self, efforts: Optional[Sequence[float]]) -> None:
        """Fold in the effort field of one ``/joint_states`` sample.

        Optional: many drivers publish an empty effort array, and a mission that
        does not ask for an effort condition never needs it. An empty or
        wrong-length array is ignored rather than raising, because it is a
        property of the driver rather than a mistake in the mission.
        """
        if not efforts or len(efforts) != 16:
            return
        self._last_efforts = [float(value) for value in efforts]

    def observe_signal(self, fired: bool, detail: str = "") -> None:
        """Fold in one reading of the external signal. Decides nothing.

        ``fired`` is the reading as the signal's own rules define it -- a
        non-empty string, a true Bool, a float past its threshold. The arming
        and the latching live here so that every condition in this module
        behaves the same way, whatever produced the reading.
        """
        self._signal_level = bool(fired)
        if detail:
            self._signal_detail = detail
        if not self._signal_armed:
            # An unarmed signal arms by being false once. An `event` signal is
            # armed at construction, so this only applies to `level`.
            if not fired:
                self._signal_armed = True
            return
        if fired:
            self._signal_fired = True

    def arm_signal_now(self) -> None:
        """Declare the signal armed without waiting to see it false.

        Used for ``event`` signals, which start unfired by definition, and for
        ``level`` signals whose mission sets ``arm: false``.
        """
        self._signal_armed = True

    def verdict(self, now: float) -> Verdict:
        """Is this phase over? Call from the loop that switches steps."""
        elapsed = now - self.start_time

        # The operator overrides everything, in any phase.
        if self._operator_aborted:
            return Verdict(True, False, "operator skipped the phase")
        if self._operator_advanced:
            return Verdict(True, True, f"operator advanced after {elapsed:.1f}s")

        # Losing proprioception mid-phase is a fault, not a slow tick: the
        # policy is still driving the arms off an observation nobody is checking.
        if self._last_state_time is not None and elapsed >= self.config.stale_grace_s:
            age = now - self._last_state_time
            if age > self.config.stale_state_s:
                return Verdict(True, False, f"joint_states stale for {age:.2f}s")

        # Every specified condition must hold at the same time. Both checks run
        # every tick whatever the other says -- each one carries state (when the
        # grasp armed, when the arm last moved) that only advances if it is
        # asked.
        wanted = 0
        met: List[str] = []
        if self.until.grasp is not None:
            wanted += 1
            reason = self._grasp_met(now, elapsed)
            if reason is not None:
                met.append(reason)
        if self.until.settled:
            wanted += 1
            reason = self._settled_met(now, elapsed)
            if reason is not None:
                met.append(reason)
        if self.until.effort is not None:
            wanted += 1
            reason = self._effort_met(now, elapsed)
            if reason is not None:
                met.append(reason)
        if self.until.signal is not None:
            wanted += 1
            reason = self._signal_met(elapsed)
            if reason is not None:
                met.append(reason)
        if self.until.envelope is not None:
            wanted += 1
            reason = self._envelope_met(now, elapsed)
            if reason is not None:
                met.append(reason)

        if wanted and len(met) == wanted:
            return Verdict(True, True, " and ".join(met))

        if elapsed >= self.until.timeout_s:
            if self.until.timeout_is_success:
                return Verdict(True, True, f"ran the full {self.until.timeout_s:.0f}s")
            return Verdict(
                True, False, f"timed out after {self.until.timeout_s:.0f}s: {self._unmet()}"
            )

        return Verdict.running()

    def update(self, now: float, positions: Optional[Sequence[float]]) -> Verdict:
        """``observe`` then ``verdict``, for a caller with only one loop."""
        if positions is not None:
            self.observe(now, positions)
        return self.verdict(now)

    # -- internals ----------------------------------------------------------

    def _observe(self, now: float, positions: List[float]) -> None:
        previous, last_time = self._last_positions, self._last_state_time
        self._last_positions = positions
        self._last_state_time = now

        # Stillness is a spread over a window, so keep exactly that window:
        # samples within hold_s of now. Trimming wider would measure over a
        # longer period than the step asked for, and the phase would end late.
        self._history.append((now, [positions[i] for i in ARM_INDICES]))
        # Keep exactly one sample at or before the window's edge, so the
        # window is bracketed rather than clipped. Dropping everything older
        # than the edge would, at a sparse or irregular sample rate, leave only
        # the last few samples and call a window full that is mostly unmeasured.
        horizon = now - self.until.hold_s
        while len(self._history) > 1 and self._history[1][0] <= horizon:
            self._history.popleft()
        if previous is None or last_time is None:
            return
        dt = now - last_time
        if dt <= 0:
            # A duplicate or out-of-order stamp. The pose is still the freshest
            # thing available, but no speed can be read from it.
            return
        speed = (
            max(abs(positions[index] - previous[index]) for index in ARM_INDICES) / dt
        )
        # Arming only: has this arm moved at all yet? A jitter spike answering
        # yes does no harm -- the question is whether the phase has started
        # doing anything, not whether it has stopped. Stillness itself is a
        # position spread; see _settled_met.
        if speed > self.config.motion_eps:
            self._motion_seen = True

    def _finger(self) -> Optional[float]:
        if self._last_positions is None or self.until.side is None:
            return None
        return self._last_positions[GRIPPER_INDEX[self.until.side]]

    def _grasp_met(self, now: float, elapsed: float) -> Optional[str]:
        """The reason the grasp condition is satisfied, or None."""
        finger = self._finger()
        if finger is None:
            return None
        if self.until.grasp == "closed":
            at_target = finger <= self.config.grasp_close_m
            at_opposite = finger >= self.config.grasp_open_m
        else:
            at_target = finger >= self.config.grasp_open_m
            at_opposite = finger <= self.config.grasp_close_m

        if not self._grasp_armed:
            if at_opposite:
                self._grasp_armed = True
            return None

        if not at_target:
            self._grasp_since = None
            return None
        if self._grasp_since is None:
            self._grasp_since = now
        if now - self._grasp_since >= self.until.hold_s:
            return (
                f"{self.until.side} gripper {self.until.grasp} ({finger:.4f} m) "
                f"held {self.until.hold_s:.1f}s at {elapsed:.1f}s"
            )
        return None

    def _settled_met(self, now: float, elapsed: float) -> Optional[str]:
        """The reason the settle condition is satisfied, or None.

        The test is a position SPREAD over the hold window: how far has the
        worst arm joint travelled in the last ``hold_s`` seconds? Teleop jitter
        has a tiny spread however fast it looks, so this distinguishes a parked
        arm from a slowly moving one, which a speed threshold cannot.
        """
        if not self._motion_seen or elapsed < self.config.settle_grace_s:
            return None
        if not self._history:
            return None
        # The window must be substantially full, or a phase could pass on two
        # samples taken a millisecond apart. Three quarters allows for the
        # sensor's own period at slow rates without letting a sliver through.
        if now - self._history[0][0] < 0.75 * self.until.hold_s:
            return None
        spread = self._spread()
        if spread is None or spread > self._still_spread:
            return None
        return (
            f"arms settled (moved {spread:.3f} rad in "
            f"{self.until.hold_s:.1f}s) at {elapsed:.1f}s"
        )

    def _spread(self) -> Optional[float]:
        """The furthest any one arm joint travelled across the window."""
        if len(self._history) < 2:
            return None
        poses = [pose for _, pose in self._history]
        return max(
            max(values) - min(values) for values in zip(*poses)
        )

    def _effort_met(self, now: float, elapsed: float) -> Optional[str]:
        """The reason the effort condition is satisfied, or None.

        No arming: an idle finger joint reads near zero, so this starts false.
        The magnitude is taken because sign is a driver convention and a grip is
        a grip whichever way the joint is being pushed.
        """
        if self._last_efforts is None or self.until.side is None:
            return None
        effort = abs(self._last_efforts[GRIPPER_INDEX[self.until.side]])
        if effort < self.until.effort:
            self._effort_since = None
            return None
        if self._effort_since is None:
            self._effort_since = now
        if now - self._effort_since >= self.until.hold_s:
            return (
                f"{self.until.side} gripper loaded ({effort:.2f} >= "
                f"{self.until.effort:.2f}) held {self.until.hold_s:.1f}s at {elapsed:.1f}s"
            )
        return None

    def _envelope_met(self, now: float, elapsed: float) -> Optional[str]:
        """The reason the envelope condition is satisfied, or None.

        Leaving the envelope resets the hold, so an arm that passes through the
        carry region on its way somewhere else does not end the phase.
        """
        if self.envelope is None or self._last_positions is None:
            return None
        if self.envelope.check(self._last_positions) is not None:
            self._envelope_since = None
            return None
        if self._envelope_since is None:
            self._envelope_since = now
        if now - self._envelope_since >= self.until.hold_s:
            return (
                f"arms inside envelope {self.until.envelope!r} for "
                f"{self.until.hold_s:.1f}s at {elapsed:.1f}s"
            )
        return None

    def _signal_met(self, elapsed: float) -> Optional[str]:
        """The reason the external signal is satisfied, or None.

        No ``hold_s`` here on purpose. A barcode read is instantaneous and a
        beam break can be brief; requiring either to persist would be requiring
        the wrong thing. Latching is what makes a short event survive until the
        verdict looks.
        """
        if not self._signal_fired:
            return None
        detail = f" ({self._signal_detail})" if self._signal_detail else ""
        return f"signal {self.until.signal!r} fired{detail} at {elapsed:.1f}s"

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
                spread = self._spread()
                if spread is None:
                    bits.append("settled: not enough joint_states to measure")
                else:
                    bits.append(
                        f"arms moved {spread:.3f} rad in the last "
                        f"{self.until.hold_s:.1f}s, wanted under "
                        f"{self._still_spread:.3f}"
                    )
        if self.until.effort is not None:
            if self._last_efforts is None:
                bits.append(
                    f"effort >= {self.until.effort:.2f} never checked -- "
                    "/joint_states carried no effort field"
                )
            else:
                reading = abs(self._last_efforts[GRIPPER_INDEX[self.until.side]])
                bits.append(
                    f"effort {self.until.side} at {reading:.2f}, wanted "
                    f">= {self.until.effort:.2f}"
                )
        if self.until.signal is not None:
            if not self._signal_armed:
                bits.append(
                    f"signal {self.until.signal!r} never armed -- it was true "
                    "from the start and never went false"
                )
            else:
                bits.append(
                    f"signal {self.until.signal!r} never fired "
                    f"(last reading {'true' if self._signal_level else 'false'})"
                )
        if self.until.envelope is not None:
            if self.envelope is None:
                bits.append(
                    f"envelope {self.until.envelope!r} was never resolved -- "
                    "it is not in arm_envelopes_file"
                )
            elif self._last_positions is None:
                bits.append(f"envelope {self.until.envelope!r}: no joint_states")
            else:
                why = self.envelope.check(self._last_positions)
                bits.append(why or f"envelope {self.until.envelope!r} held too briefly")
        if self.until.operator:
            bits.append("no operator advance")
        return "; ".join(bits) or "no early condition set"
