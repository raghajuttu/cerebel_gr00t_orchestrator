"""Quantitative base moves by wheel-arc integration -- Nav2's stand-in.

Nav2 cannot run on this chassis yet. It closes its loop on ``nav_msgs/Odometry``
plus an ``odom -> base_link`` transform, and the vendor stack publishes neither:
the only feedback is ``/all_wheel_rpm``, four wheel speeds. Those cannot be
turned into a pose, because this is a **swerve** chassis -- wheel speeds alone
never determine a steered robot's motion without the steer angles, and the steer
angles are not on the bus.

What they *can* give is distance travelled. That is enough for this task: three
stations on one line, a fixed distance apart, reached by strafing. So a move here
is "drive left until the wheels have turned 38 cm worth of arc, then stop" --
the same thing ``move_base/move_base.py`` does, which is the script this logic is
taken from, including its measured scale factors and its failure detection.

**This is dead reckoning and it does not correct itself.** There is no absolute
reference anywhere in the loop. What it does do is track the *achieved* arc of
every move rather than the commanded one, so a move that comes up short leaves
the next move a correspondingly longer one rather than silently shifting every
station that follows. That handles the part of the error the wheels can see. It
does not handle slip they cannot, and it does not handle heading drift at all.
docs/NAVIGATION.md lists what to add, in order; a fiducial at each station is the
one that matters most, because it puts an absolute measurement exactly where the
centimetres count.

## Shape

Poll-shaped and split in two, like ``ArmParker``:

* ``observe`` folds in one ``/all_wheel_rpm`` sample. Called from the
  subscription, at whatever rate the chassis publishes.
* ``step`` produces the twist for this tick. Called from the execution loop.
* ``outcome`` answers "is the move over?". Called from the supervision loop.

Integrating in ``observe`` rather than sampling in ``step`` is the same
discipline as ``PhaseMonitor``: the arc is a rate times an interval, and the
interval has to be the sensor's, not a loop's. Tuned against a 20 Hz feedback
stream, an arc computed on the execution loop's ticks would mean something
different the moment either rate changed.

## Who stops the wheels

Not this module, and that is deliberate. The chassis has **no command
watchdog** -- a nonzero twist drives until something sends a zero. ``step``
simply stops producing twists when the move is done, and ``base_adapter`` shuts
its gate and publishes zeros at ``zero_rate_hz``, fail-closed on a 0.5 s
heartbeat. That holds even if this node crashes mid-move, which is the case a
stop-on-exit path in here would not cover.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# --- chassis constants -------------------------------------------------------
# From chassis_control/config/params.yaml, via move_base/move_base.py.
WHEEL_PERIMETER_M = 0.647        # metres of ground per wheel revolution

# Below this the vendor driver forces the wheel to zero, so it is also the
# threshold for "the chassis is actually moving".
MOVING_RPM = 1.0

# Guards, all from move_base.py where they were arrived at on hardware.
MAX_DT_S = 0.2                   # cap one integration interval; packet loss
FEEDBACK_GAP_S = 0.5             # no sample for this long mid-move -> abort
STALE_LIMIT_S = 1.0              # identical samples while moving -> CAN down
NO_FEEDBACK_START_S = 2.0        # never heard from the chassis at all -> abort


@dataclass(frozen=True)
class BaseMoveConfig:
    """Chassis properties. Not mission properties -- these live in params.

    ``scale`` is the calibration coefficient, measured divided by integrated.
    It differs per axis because the mechanics do: ``move_base.py`` measured 1.02
    forward and back, 1.05 left and right. The crab correction is **not** here;
    it is a property of the twist rather than of the distance, so it lives in
    ``base_adapter`` where the last word on what reaches the wheels is.
    """

    speed_mps: float = 0.10          # cruise speed for a move
    min_speed_mps: float = 0.03      # floor during the taper; below this the
                                     # driver's 1 rpm deadband truncates to zero
    taper_m: float = 0.15            # start easing off this far from target
    taper_floor: float = 0.35        # never taper below this fraction of speed
    scale_lateral: float = 1.05      # measured / integrated, left and right
    scale_axial: float = 1.02        # measured / integrated, forward and back
    tolerance_m: float = 0.01        # how close counts as arrived

    def validate(self) -> None:
        if self.speed_mps <= 0 or self.min_speed_mps <= 0:
            raise ValueError("base move speeds must be positive")
        if self.min_speed_mps > self.speed_mps:
            raise ValueError(
                f"min_speed_mps ({self.min_speed_mps}) is above speed_mps "
                f"({self.speed_mps}); the taper would speed the base up"
            )
        if self.taper_m < 0:
            raise ValueError("taper_m must be >= 0 (0 disables the taper)")
        if not 0.0 < self.taper_floor <= 1.0:
            raise ValueError("taper_floor must be in (0, 1]")
        if self.scale_lateral <= 0 or self.scale_axial <= 0:
            raise ValueError("scale factors must be positive")
        if self.tolerance_m <= 0:
            raise ValueError("tolerance_m must be positive")


# Unit twist direction per axis sign. Lateral is +y left, axial is +x forward,
# matching move_base.py's DIRECTIONS table and the ROS convention.
_AXIS_VECTORS = {
    ("lateral", 1): (0.0, 1.0),      # left
    ("lateral", -1): (0.0, -1.0),    # right
    ("axial", 1): (1.0, 0.0),        # forward
    ("axial", -1): (-1.0, 0.0),      # back
}


class BaseMover:
    """One quantitative move along one axis.

    ``now`` is supplied by the caller on every call, as everywhere else in this
    package: the node passes the ROS clock, the tests pass floats.
    """

    def __init__(self, config: BaseMoveConfig) -> None:
        config.validate()
        self.config = config
        self._active = False
        self._axis = "lateral"
        self._sign = 1
        self._target_m = 0.0
        self._travelled_m = 0.0
        self._scale = config.scale_lateral
        self._started_at: Optional[float] = None
        self._last_sample_at: Optional[float] = None
        self._last_values: Optional[List[float]] = None
        self._stale_since: Optional[float] = None
        self._done: Optional[Tuple[bool, str]] = None

    # -- one move ------------------------------------------------------------

    def start(self, distance_m: float, axis: str, now: float) -> Optional[str]:
        """Begin a move. Returns an error string if the request is unusable.

        ``distance_m`` is signed: positive is left (lateral) or forward (axial).
        A distance inside the tolerance is not an error -- it is a move that is
        already finished, and it completes on the first ``outcome``.
        """
        if axis not in ("lateral", "axial"):
            return f"unknown base-move axis {axis!r}; expected 'lateral' or 'axial'"
        self._active = True
        self._axis = axis
        self._sign = 1 if distance_m >= 0 else -1
        self._target_m = abs(distance_m)
        self._travelled_m = 0.0
        self._scale = (
            self.config.scale_lateral if axis == "lateral" else self.config.scale_axial
        )
        self._started_at = now
        self._last_sample_at = None
        self._last_values = None
        self._stale_since = None
        self._done = None
        if self._target_m <= self.config.tolerance_m:
            self._done = (True, f"already within {self.config.tolerance_m * 100:.1f} cm")
        return None

    @property
    def active(self) -> bool:
        return self._active and self._done is None

    @property
    def travelled_m(self) -> float:
        """Signed arc achieved so far, in the axis's own direction."""
        return self._sign * self._travelled_m

    @property
    def remaining_m(self) -> float:
        return max(0.0, self._target_m - self._travelled_m)

    def cancel(self) -> None:
        """Stop producing twists. The adapter's gate does the actual stopping."""
        self._active = False
        self._done = None

    # -- observation ---------------------------------------------------------

    def observe(self, now: float, wheel_rpm: Sequence[float]) -> None:
        """Fold in one ``/all_wheel_rpm`` sample. Decides nothing.

        The four signs are in motor coordinates and ``move_base.py`` records
        that they are inconsistent between translation and rotation, with the
        front-wheel indices in doubt. So this takes the mean of the absolute
        values, which is valid for both: in a pure translation all four wheels
        turn at the same speed, and so do they in a spin about the centre. It
        is an odometer, not odometry -- it knows how far, never which way.
        """
        if not self.active:
            return
        values = [float(v) for v in list(wheel_rpm)[:4]]
        if not values:
            return

        mean_rpm = sum(abs(v) for v in values) / len(values)
        spinning = mean_rpm > MOVING_RPM

        if self._last_sample_at is not None:
            dt = now - self._last_sample_at
            if 0.0 < dt < MAX_DT_S:
                self._travelled_m += (mean_rpm / 60.0) * WHEEL_PERIMETER_M * self._scale * dt

        # A dropped CAN link leaves the rpm publisher repeating its last frame
        # forever. Integrating that drives the base until the target is "met",
        # with the wheels doing whatever they were last told to. Catch it.
        if spinning and values == self._last_values:
            if self._stale_since is None:
                self._stale_since = now
            elif now - self._stale_since > STALE_LIMIT_S:
                self._done = (
                    False,
                    "wheel feedback frozen -- identical rpm for "
                    f"{STALE_LIMIT_S:.1f}s while moving; CAN may be down",
                )
        else:
            self._stale_since = None

        self._last_sample_at = now
        self._last_values = values

    # -- the twist for this tick ---------------------------------------------

    def step(self, now: float) -> Optional[Tuple[float, float]]:
        """``(linear_x, linear_y)`` for this tick, or None if nothing to send.

        Returning None is how a move stops: no twist is published, the gate
        shuts, and ``base_adapter`` zeroes the wheels. This never publishes a
        stop itself -- see the module docstring.
        """
        if not self.active:
            return None

        self._check_feedback(now)
        if self._done is not None:
            return None

        if self._travelled_m >= self._target_m:
            achieved_cm = self._travelled_m * 100.0
            self._done = (
                True,
                f"moved {achieved_cm:.1f} cm of {self._target_m * 100.0:.1f} cm",
            )
            return None

        speed = self.config.speed_mps
        if self.config.taper_m > 0.0 and self.remaining_m < self.config.taper_m:
            ratio = max(self.remaining_m / self.config.taper_m, self.config.taper_floor)
            speed = max(speed * ratio, self.config.min_speed_mps)

        unit_x, unit_y = _AXIS_VECTORS[(self._axis, self._sign)]
        return (unit_x * speed, unit_y * speed)

    # -- the verdict ---------------------------------------------------------

    def outcome(self, now: float) -> Optional[Tuple[bool, str]]:
        """None while the move is running; ``(ok, reason)`` once it is not."""
        if not self._active:
            return None
        self._check_feedback(now)
        return self._done

    def _check_feedback(self, now: float) -> None:
        """Abort rather than drive blind. The chassis has no watchdog."""
        if self._done is not None or self._started_at is None:
            return
        if self._last_sample_at is None:
            if now - self._started_at > NO_FEEDBACK_START_S:
                self._done = (
                    False,
                    f"no wheel feedback {NO_FEEDBACK_START_S:.0f}s after the move began "
                    "-- is the chassis driver up?",
                )
            return
        if now - self._last_sample_at > FEEDBACK_GAP_S:
            self._done = (
                False,
                f"wheel feedback stopped {now - self._last_sample_at:.1f}s ago, "
                f"{self.remaining_m * 100.0:.1f} cm short",
            )
