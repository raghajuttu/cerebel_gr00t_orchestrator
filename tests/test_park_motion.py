"""The park move is smooth, and one mission can enable one park.

A linear ramp jumps to full speed on its first tick and stops dead on its last;
the arm shows that as a snap at both ends. The park is now timed by a
minimum-jerk profile, and the atomic missions' go-to-ready step moves the arms
with enable_park still false everywhere else.
"""

import pytest

from cerebel_orchestrator.mission import Mission, MissionError, Step


def test_park_arms_takes_enable():
    step = Step.parse(0, {"step": "park_arms", "profile": "ready_atomic", "enable": True})
    assert step.park_enable
    assert "enabled by the mission" in step.describe()
    assert not Step.parse(0, {"step": "park_arms", "profile": "home"}).park_enable


def test_park_arms_enable_must_be_a_bool():
    with pytest.raises(MissionError, match="true or false"):
        Step.parse(0, {"step": "park_arms", "profile": "home", "enable": "yes"})


def test_the_atomic_missions_go_to_ready_first():
    for path in ("missions/atomic_kit.yaml", "missions/atomic_kit_one.yaml"):
        first = Mission.load(path).steps[0]
        assert first.kind == "park_arms"
        assert first.profile == "ready_atomic"
        assert first.park_enable


def arm_park():
    """The module itself imports rclpy, so these run on the robot only."""
    return pytest.importorskip("cerebel_orchestrator.arm_park")


def test_min_jerk_starts_and_ends_at_rest():
    f = arm_park().min_jerk
    assert f(0.0) == 0.0 and f(1.0) == 1.0
    assert f(0.5) == pytest.approx(0.5)
    # Speed near both ends is tiny: no snap into or out of the move.
    eps = 1e-3
    assert (f(eps) - f(0.0)) / eps < 1e-4
    assert (f(1.0) - f(1.0 - eps)) / eps < 1e-4
    # Clamped outside 0..1.
    assert f(-1.0) == 0.0 and f(2.0) == 1.0


def test_min_jerk_peak_speed_is_the_constant_the_duration_uses():
    module = arm_park()
    f = module.min_jerk
    eps = 1e-6
    peak = (f(0.5 + eps) - f(0.5 - eps)) / (2 * eps)
    assert peak == pytest.approx(module.MIN_JERK_PEAK, rel=1e-4)
