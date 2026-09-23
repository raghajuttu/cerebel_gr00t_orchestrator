"""The two loops: observing at sensor rate, deciding at supervisor rate.

The orchestrator runs an execution loop (sustains motion, publishes the gate
heartbeat) and a supervisor loop (asks whether the step is done, switches). The
completion check is fed from the ``/joint_states`` subscription rather than
sampled by either loop, which is what makes its thresholds independent of both
rates. These tests pin that independence, because it is the kind of property that
holds until somebody changes a rate and then quietly does not.
"""

import pytest

from cerebel_orchestrator.joints import GRIPPER_INDEX
from cerebel_orchestrator.mission import Until
from cerebel_orchestrator.phase_monitor import MonitorConfig, PhaseMonitor

CFG = MonitorConfig(
    grasp_close_m=0.010,
    grasp_open_m=0.030,
    motion_eps=0.05,  # rad/s
    settle_grace_s=1.0,
)


def state(left_finger=0.04, arm=0.0):
    positions = [arm] * 16
    positions[GRIPPER_INDEX["left"]] = left_finger
    positions[GRIPPER_INDEX["right"]] = 0.04
    return positions


def run(sensor_hz, supervisor_hz, move_for_s, then_hold_s, until, config=CFG):
    """Feed a synthetic run at two independent rates; return the first verdict.

    The arm moves at 0.5 rad/s for ``move_for_s`` and then holds still. The
    monitor is observed at ``sensor_hz`` and asked for a verdict at
    ``supervisor_hz``, exactly as the node does it.
    """
    monitor = PhaseMonitor(until, config, start_time=0.0)
    sensor_dt = 1.0 / sensor_hz
    supervisor_dt = 1.0 / supervisor_hz
    next_sensor = 0.0
    next_supervisor = supervisor_dt
    t = 0.0
    total = move_for_s + then_hold_s
    angle = 0.0
    while t <= total:
        t = round(min(next_sensor, next_supervisor), 9)
        if t >= next_sensor - 1e-12:
            angle = 0.5 * t if t <= move_for_s else 0.5 * move_for_s
            finger = 0.040 if t < move_for_s else 0.005
            monitor.observe(t, state(left_finger=finger, arm=angle))
            next_sensor = round(next_sensor + sensor_dt, 9)
        if t >= next_supervisor - 1e-12:
            verdict = monitor.verdict(t)
            next_supervisor = round(next_supervisor + supervisor_dt, 9)
            if verdict.done:
                return t, verdict
    return None, monitor.verdict(total)


@pytest.mark.parametrize("sensor_hz", [10.0, 30.0, 100.0])
@pytest.mark.parametrize("supervisor_hz", [5.0, 10.0, 50.0])
def test_settle_fires_at_the_same_time_whatever_the_rates(sensor_hz, supervisor_hz):
    until = Until(timeout_s=60, settled=True, hold_s=0.5)
    at, verdict = run(sensor_hz, supervisor_hz, move_for_s=4.0, then_hold_s=4.0, until=until)
    assert verdict.done and verdict.ok
    # The arm stops at 4.0 s and must be declared settled hold_s later, within
    # one period of each loop. A per-tick-delta threshold would instead fire at
    # wildly different times -- or never, at a fast sensor rate where each
    # sample's delta is small.
    assert at == pytest.approx(4.5, abs=1.0 / min(sensor_hz, supervisor_hz) + 0.05)


@pytest.mark.parametrize("sensor_hz", [10.0, 30.0, 100.0])
def test_a_moving_arm_is_never_called_settled_at_any_sensor_rate(sensor_hz):
    """The bug this design removes.

    At 100 Hz an arm moving 0.5 rad/s covers 0.005 rad per sample. Against a
    per-tick-delta threshold of 0.05 that reads as stationary, and the phase
    would end with the arm in motion. Against a speed it reads as 0.5 rad/s.
    """
    until = Until(timeout_s=6.0, settled=True, hold_s=0.5)
    at, verdict = run(sensor_hz, 10.0, move_for_s=7.0, then_hold_s=0.0, until=until)
    # The only thing that ends this phase is the clock -- never the settle check.
    assert at == pytest.approx(6.0, abs=0.2)
    assert verdict.done and not verdict.ok
    assert "never stopped moving" in verdict.reason


def test_observing_without_deciding_changes_nothing():
    until = Until(timeout_s=60, settled=True, hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    for i in range(200):  # 20 s of observations, no verdict ever asked for
        t = i * 0.1
        monitor.observe(t, state(arm=0.5 * min(t, 4.0)))
    # The state accumulated; the first verdict sees it all at once.
    verdict = monitor.verdict(20.0)
    assert verdict.done and verdict.ok


def test_deciding_without_observing_falls_through_to_the_timeout():
    until = Until(timeout_s=5.0, grasp="closed", side="left")
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    for t in (1.0, 2.0, 3.0, 4.0):
        assert not monitor.verdict(t).done
    verdict = monitor.verdict(5.1)
    assert verdict.done and not verdict.ok and "no joint_states" in verdict.reason


def test_a_duplicate_timestamp_does_not_divide_by_zero():
    monitor = PhaseMonitor(Until(timeout_s=60, settled=True), CFG, start_time=0.0)
    monitor.observe(1.0, state(arm=0.0))
    monitor.observe(1.0, state(arm=1.0))  # same stamp, different pose
    monitor.observe(0.5, state(arm=2.0))  # and one out of order
    assert not monitor.verdict(1.0).done


def test_the_stale_check_is_suppressed_during_the_startup_grace():
    """A client warming up can starve the subscription; that is not a failure."""
    config = MonitorConfig(motion_eps=0.05, stale_state_s=1.0, stale_grace_s=10.0)
    monitor = PhaseMonitor(Until(timeout_s=60, settled=True), config, start_time=0.0)
    monitor.observe(0.0, state())
    # Five seconds without a sample: inside the grace, so the phase survives.
    assert not monitor.verdict(5.0).done
    # Past the grace, the same gap is a failure.
    verdict = monitor.verdict(11.0)
    assert verdict.done and not verdict.ok and "stale" in verdict.reason
