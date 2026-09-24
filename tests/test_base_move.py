"""The wheel-arc mover: integration, taper, and every way a move can go wrong.

No ROS. ``BaseMover`` takes its clock from the caller, so a move can be driven
here one sample at a time and the interesting cases -- a CAN link that dies
mid-move, feedback that never arrives, a move that comes up short -- are
answered without a robot.
"""

import pytest

from cerebel_orchestrator.base_move import (
    FEEDBACK_GAP_S,
    MOVING_RPM,
    NO_FEEDBACK_START_S,
    STALE_LIMIT_S,
    WHEEL_PERIMETER_M,
    BaseMoveConfig,
    BaseMover,
)

# A speed that is comfortably above the driver's 1 rpm deadband.
CRUISE_RPM = 10.0


def config(**overrides) -> BaseMoveConfig:
    base = dict(speed_mps=0.10, min_speed_mps=0.03, taper_m=0.0, tolerance_m=0.01)
    base.update(overrides)
    return BaseMoveConfig(**base)


def drive(mover: BaseMover, *, rpm: float, seconds: float, start: float = 0.0,
          dt: float = 0.05) -> float:
    """Feed samples at ``dt`` until ``seconds`` have passed. Returns the clock."""
    now = start
    steps = int(round(seconds / dt))
    for _ in range(steps):
        now += dt
        mover.observe(now, [rpm, -rpm, -rpm, -rpm])
        mover.step(now)
    return now


# -- configuration -----------------------------------------------------------


def test_config_rejects_a_taper_that_speeds_the_base_up():
    with pytest.raises(ValueError, match="taper would speed the base up"):
        BaseMoveConfig(speed_mps=0.05, min_speed_mps=0.10).validate()


def test_config_rejects_nonsense_scales():
    with pytest.raises(ValueError, match="scale factors"):
        BaseMoveConfig(scale_lateral=0.0).validate()


def test_unknown_axis_is_rejected_before_anything_moves():
    mover = BaseMover(config())
    error = mover.start(0.38, "diagonal", now=0.0)
    assert error is not None and "diagonal" in error
    assert mover.step(0.1) is None


# -- integration -------------------------------------------------------------


def test_arc_integrates_to_the_commanded_distance():
    # One wheel revolution per second is WHEEL_PERIMETER_M of ground per second.
    rpm = 60.0
    mover = BaseMover(config(scale_lateral=1.0))
    assert mover.start(WHEEL_PERIMETER_M, "lateral", now=0.0) is None

    now = drive(mover, rpm=rpm, seconds=0.9)
    assert mover.outcome(now) is None, "should still be short of the target"

    now = drive(mover, rpm=rpm, seconds=0.3, start=now)
    outcome = mover.outcome(now)
    assert outcome is not None and outcome[0] is True
    assert mover.travelled_m == pytest.approx(WHEEL_PERIMETER_M, abs=0.05)


def test_the_scale_factor_is_applied():
    mover = BaseMover(config(scale_lateral=2.0))
    mover.start(10.0, "lateral", now=0.0)          # far enough not to finish
    drive(mover, rpm=60.0, seconds=1.0)
    # Twice the ground per revolution, because scale doubles it.
    assert mover.travelled_m == pytest.approx(2 * WHEEL_PERIMETER_M, abs=0.1)


def test_the_axis_chooses_its_own_scale():
    mover = BaseMover(config(scale_lateral=1.0, scale_axial=2.0))
    mover.start(10.0, "axial", now=0.0)
    drive(mover, rpm=60.0, seconds=1.0)
    assert mover.travelled_m == pytest.approx(2 * WHEEL_PERIMETER_M, abs=0.1)


def test_travelled_is_signed_but_distance_is_not():
    mover = BaseMover(config(scale_lateral=1.0))
    mover.start(-10.0, "lateral", now=0.0)         # a move to the right
    drive(mover, rpm=60.0, seconds=1.0)
    assert mover.travelled_m < 0, "a rightward move must book a negative arc"


def test_a_long_gap_between_samples_is_not_integrated():
    """A dropped packet must not be read as a metre of travel."""
    mover = BaseMover(config(scale_lateral=1.0))
    mover.start(10.0, "lateral", now=0.0)
    mover.observe(0.0, [60.0] * 4)
    mover.observe(5.0, [60.0] * 4)                 # 5 s gap, way over MAX_DT_S
    assert mover.travelled_m == pytest.approx(0.0)


# -- direction ---------------------------------------------------------------


@pytest.mark.parametrize(
    "distance, axis, expected",
    [
        (0.38, "lateral", (0.0, 1.0)),     # left
        (-0.38, "lateral", (0.0, -1.0)),   # right
        (0.38, "axial", (1.0, 0.0)),       # forward
        (-0.38, "axial", (-1.0, 0.0)),     # back
    ],
)
def test_the_twist_points_the_right_way(distance, axis, expected):
    mover = BaseMover(config(speed_mps=0.2))
    mover.start(distance, axis, now=0.0)
    mover.observe(0.0, [CRUISE_RPM] * 4)
    twist = mover.step(0.05)
    assert twist is not None
    unit_x, unit_y = expected
    assert twist == pytest.approx((unit_x * 0.2, unit_y * 0.2))


def taper_speed(target_m: float, **overrides) -> float:
    """The lateral speed on the first tick of a move this far from its target.

    Driving into the taper would couple this to the integration, so the target
    is simply set inside the taper zone to begin with -- the taper only ever
    looks at what remains.
    """
    mover = BaseMover(config(taper_m=0.20, scale_lateral=1.0, **overrides))
    mover.start(target_m, "lateral", now=0.0)
    mover.observe(0.0, [CRUISE_RPM] * 4)
    twist = mover.step(0.05)
    assert twist is not None
    return twist[1]


def test_outside_the_taper_the_base_cruises():
    assert taper_speed(1.00, speed_mps=0.20, min_speed_mps=0.05) == pytest.approx(0.20)


def test_inside_the_taper_the_base_slows_proportionally():
    # 0.10 m remaining of a 0.20 m taper -> half speed.
    assert taper_speed(0.10, speed_mps=0.20, min_speed_mps=0.05) == pytest.approx(0.10)


def test_the_taper_never_goes_below_the_floor():
    """Below the 1 rpm deadband the driver truncates to zero and the base
    stops short of the target with no error -- so the taper has a floor."""
    crawl = taper_speed(0.02, speed_mps=0.20, min_speed_mps=0.10)
    assert crawl == pytest.approx(0.10), "min_speed_mps must win over the ratio"


def test_taper_zero_disables_it():
    mover = BaseMover(config(speed_mps=0.20, taper_m=0.0, scale_lateral=1.0))
    mover.start(0.02, "lateral", now=0.0)
    mover.observe(0.0, [CRUISE_RPM] * 4)
    twist = mover.step(0.05)
    assert twist is not None and twist[1] == pytest.approx(0.20)


# -- stopping ----------------------------------------------------------------


def test_a_finished_move_publishes_nothing():
    """The mover never sends a stop -- base_adapter's gate does that."""
    mover = BaseMover(config(scale_lateral=1.0))
    mover.start(0.1, "lateral", now=0.0)
    now = drive(mover, rpm=60.0, seconds=1.0)
    assert mover.outcome(now)[0] is True
    assert mover.step(now + 0.05) is None


def test_a_distance_inside_the_tolerance_is_already_done():
    mover = BaseMover(config(tolerance_m=0.02))
    mover.start(0.005, "lateral", now=0.0)
    outcome = mover.outcome(0.0)
    assert outcome is not None and outcome[0] is True
    assert mover.step(0.05) is None


def test_cancel_stops_producing_twists():
    mover = BaseMover(config())
    mover.start(1.0, "lateral", now=0.0)
    mover.observe(0.0, [CRUISE_RPM] * 4)
    assert mover.step(0.05) is not None
    mover.cancel()
    assert mover.step(0.10) is None
    assert mover.outcome(0.10) is None


# -- the ways a move fails ---------------------------------------------------


def test_frozen_feedback_aborts_the_move():
    """A dead CAN link leaves the rpm publisher repeating its last frame.

    Integrating that would drive the base until the target was nominally met,
    with the wheels doing whatever they were last told to -- the single most
    dangerous failure available to this chassis.
    """
    mover = BaseMover(config(scale_lateral=1.0))
    mover.start(10.0, "lateral", now=0.0)
    frozen = [CRUISE_RPM, -CRUISE_RPM, -CRUISE_RPM, -CRUISE_RPM]
    now = 0.0
    for _ in range(int((STALE_LIMIT_S + 0.5) / 0.05)):
        now += 0.05
        mover.observe(now, list(frozen))
    outcome = mover.outcome(now)
    assert outcome is not None and outcome[0] is False
    assert "frozen" in outcome[1]
    assert mover.step(now + 0.05) is None, "a frozen-feedback move must stop driving"


def test_identical_samples_while_stationary_are_not_a_freeze():
    """Below the deadband the wheels are not turning, so repeats are expected."""
    mover = BaseMover(config())
    mover.start(10.0, "lateral", now=0.0)
    still = [MOVING_RPM / 2.0] * 4
    now = 0.0
    for _ in range(int((STALE_LIMIT_S + 1.0) / 0.05)):
        now += 0.05
        mover.observe(now, list(still))
    assert mover.outcome(now) is None


def test_feedback_that_never_arrives_aborts():
    mover = BaseMover(config())
    mover.start(1.0, "lateral", now=0.0)
    assert mover.outcome(NO_FEEDBACK_START_S - 0.5) is None
    outcome = mover.outcome(NO_FEEDBACK_START_S + 0.5)
    assert outcome is not None and outcome[0] is False
    assert "chassis driver" in outcome[1]


def test_feedback_that_stops_mid_move_aborts_and_says_how_far_short():
    mover = BaseMover(config(scale_lateral=1.0))
    mover.start(10.0, "lateral", now=0.0)
    now = drive(mover, rpm=60.0, seconds=1.0)
    assert mover.outcome(now) is None

    outcome = mover.outcome(now + FEEDBACK_GAP_S + 0.1)
    assert outcome is not None and outcome[0] is False
    assert "cm short" in outcome[1]


def test_a_failed_move_still_reports_the_distance_it_achieved():
    """The orchestrator books this against the axis: a move that failed
    halfway still moved the robot halfway, and the next step has to know."""
    mover = BaseMover(config(scale_lateral=1.0))
    mover.start(10.0, "lateral", now=0.0)
    now = drive(mover, rpm=60.0, seconds=1.0)
    mover.outcome(now + FEEDBACK_GAP_S + 0.1)
    assert mover.travelled_m == pytest.approx(WHEEL_PERIMETER_M, abs=0.05)
