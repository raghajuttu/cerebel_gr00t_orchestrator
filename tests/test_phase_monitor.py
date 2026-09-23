"""Phase termination: the grasp check, the settle check, and the arming rules."""

import pytest

from cerebel_orchestrator.joints import LEFT_GRIPPER, RIGHT_GRIPPER
from cerebel_orchestrator.mission import Until
from cerebel_orchestrator.phase_monitor import MonitorConfig, PhaseMonitor

CFG = MonitorConfig(
    grasp_close_m=0.010, grasp_open_m=0.030, motion_eps=0.004, settle_grace_s=3.0
)


def state(left_finger=0.04, right_finger=0.04, arm=0.0):
    positions = [arm] * 16
    positions[LEFT_GRIPPER] = left_finger
    positions[RIGHT_GRIPPER] = right_finger
    return positions


def test_closed_grasp_needs_the_open_state_first():
    until = Until(timeout_s=30, grasp="closed", side="left", hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    # Starts already closed: that is the previous episode's pose, not a grasp.
    for t in range(0, 20):
        verdict = monitor.update(t * 0.1, state(left_finger=0.002))
        assert not verdict.done


def test_closed_grasp_fires_after_open_then_closed_held():
    until = Until(timeout_s=30, grasp="closed", side="left", hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    assert not monitor.update(0.0, state(left_finger=0.040)).done  # arms
    assert not monitor.update(0.1, state(left_finger=0.005)).done  # hold starts
    assert not monitor.update(0.4, state(left_finger=0.005)).done  # not long enough
    verdict = monitor.update(0.7, state(left_finger=0.005))
    assert verdict.done and verdict.ok
    assert "left gripper closed" in verdict.reason


def test_a_bounce_out_of_the_threshold_restarts_the_hold():
    until = Until(timeout_s=30, grasp="closed", side="left", hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    monitor.update(0.0, state(left_finger=0.040))
    monitor.update(0.1, state(left_finger=0.005))
    monitor.update(0.3, state(left_finger=0.020))  # slipped
    assert not monitor.update(0.7, state(left_finger=0.005)).done
    assert monitor.update(1.3, state(left_finger=0.005)).done


def test_the_other_side_is_not_watched():
    until = Until(timeout_s=30, grasp="closed", side="right", hold_s=0.2)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    monitor.update(0.0, state(right_finger=0.040))
    # The left gripper closing is not this phase's business.
    assert not monitor.update(0.5, state(left_finger=0.001, right_finger=0.040)).done
    assert not monitor.update(1.0, state(left_finger=0.040, right_finger=0.001)).done
    assert monitor.update(1.3, state(left_finger=0.040, right_finger=0.001)).done


def test_open_grasp_is_the_mirror_case():
    until = Until(timeout_s=30, grasp="open", side="left", hold_s=0.2)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    assert not monitor.update(0.0, state(left_finger=0.002)).done  # arms
    monitor.update(0.1, state(left_finger=0.040))
    verdict = monitor.update(0.4, state(left_finger=0.040))
    assert verdict.done and verdict.ok


def test_settled_does_not_fire_before_the_arms_have_moved():
    until = Until(timeout_s=30, settled=True, hold_s=1.0)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    for i in range(100):  # 10 s of a perfectly still arm
        assert not monitor.update(i * 0.1, state(arm=0.0)).done


def test_settled_fires_after_motion_stops():
    until = Until(timeout_s=60, settled=True, hold_s=1.0)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    t = 0.0
    for i in range(40):  # 4 s of motion, well clear of motion_eps
        t = i * 0.1
        monitor.update(t, state(arm=i * 0.05))
    held = state(arm=40 * 0.05)
    # The first still tick only ends the motion; the hold window opens on the
    # second, which is when the arm is known to have stayed put.
    assert not monitor.update(t + 0.1, held).done
    assert not monitor.update(t + 0.2, held).done
    assert not monitor.update(t + 1.0, held).done
    verdict = monitor.update(t + 1.4, held)
    assert verdict.done and verdict.ok and "settled" in verdict.reason


def test_settle_grace_holds_off_an_early_pause():
    until = Until(timeout_s=60, settled=True, hold_s=0.2)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    monitor.update(0.0, state(arm=0.0))
    monitor.update(0.1, state(arm=0.5))  # motion seen at once
    for t in (0.5, 1.0, 2.0, 2.9):
        assert not monitor.update(t, state(arm=0.5)).done
    assert monitor.update(3.1, state(arm=0.5)).done


def test_timeout_only_phase_succeeds_on_the_clock():
    until = Until(timeout_s=5.0)
    monitor = PhaseMonitor(until, CFG, start_time=100.0)
    assert not monitor.update(104.0, state()).done
    verdict = monitor.update(105.0, state())
    assert verdict.done and verdict.ok and "full 5s" in verdict.reason


def test_timeout_with_an_unmet_condition_fails_and_says_why():
    until = Until(timeout_s=5.0, grasp="closed", side="left", hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    monitor.update(0.1, state(left_finger=0.020))  # between the thresholds
    verdict = monitor.update(5.1, state(left_finger=0.020))
    assert verdict.done and not verdict.ok
    assert "never armed" in verdict.reason and "0.0200" in verdict.reason


def test_stale_joint_states_end_the_phase_as_a_failure():
    until = Until(timeout_s=60, grasp="closed", side="left")
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    monitor.update(0.0, state())
    assert not monitor.update(0.9, None).done
    verdict = monitor.update(1.6, None)
    assert verdict.done and not verdict.ok and "stale" in verdict.reason


def test_no_joint_states_at_all_still_times_out():
    until = Until(timeout_s=2.0, grasp="closed", side="left")
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    assert not monitor.update(1.0, None).done
    verdict = monitor.update(2.5, None)
    assert verdict.done and not verdict.ok
    assert "no joint_states" in verdict.reason


def test_operator_advance_and_skip():
    monitor = PhaseMonitor(Until(timeout_s=60, operator=True), CFG, 0.0)
    assert not monitor.update(1.0, state()).done
    monitor.operator_advance()
    verdict = monitor.update(2.0, state())
    assert verdict.done and verdict.ok

    other = PhaseMonitor(Until(timeout_s=60, operator=True), CFG, 0.0)
    other.operator_skip()
    verdict = other.update(1.0, state())
    assert verdict.done and not verdict.ok


def test_config_and_state_shape_are_checked():
    with pytest.raises(ValueError, match="grasp_close_m"):
        MonitorConfig(grasp_close_m=0.05, grasp_open_m=0.01).validate()
    monitor = PhaseMonitor(Until(timeout_s=5), CFG, 0.0)
    with pytest.raises(ValueError, match="16 canonical joints"):
        monitor.update(0.0, [0.0] * 14)


# -- conditions combine with AND (the pick-and-carry check) ------------------


def test_grasp_and_settled_both_have_to_hold():
    """The carry check: object held AND the arm has come to rest."""
    until = Until(timeout_s=60, grasp="closed", side="left", settled=True, hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)

    # Reaching for the object, gripper open: arms the grasp, arms the settle.
    t = 0.0
    for i in range(40):
        t = i * 0.1
        assert not monitor.update(t, state(left_finger=0.040, arm=i * 0.05)).done

    # Gripper closes while the arm is still moving -- a grasp mid-reach must NOT
    # end the phase, because the base would drive off with the arm swinging.
    for i in range(40, 55):
        t = i * 0.1
        assert not monitor.update(t, state(left_finger=0.005, arm=i * 0.05)).done

    # Now the arm comes to rest holding the object.
    held = state(left_finger=0.005, arm=54 * 0.05)
    monitor.update(t + 0.1, held)
    monitor.update(t + 0.2, held)
    verdict = monitor.update(t + 1.0, held)
    assert verdict.done and verdict.ok
    assert "gripper closed" in verdict.reason and "settled" in verdict.reason


def test_settling_without_the_object_does_not_end_the_phase():
    until = Until(timeout_s=8.0, grasp="closed", side="left", settled=True, hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    for i in range(40):  # moves, gripper stays open: the grasp never fires
        monitor.update(i * 0.1, state(left_finger=0.040, arm=i * 0.05))
    still = state(left_finger=0.040, arm=39 * 0.05)
    for t in (4.1, 4.2, 5.0, 6.0, 7.0):
        assert not monitor.update(t, still).done
    verdict = monitor.update(8.1, still)
    assert verdict.done and not verdict.ok
    assert "grasp left closed armed" in verdict.reason


def test_a_closing_gripper_is_not_an_arm_still_moving():
    """The fingers are excluded from the settle check -- and are in metres."""
    until = Until(timeout_s=60, settled=True, hold_s=0.5)
    monitor = PhaseMonitor(until, CFG, start_time=0.0)
    t = 0.0
    for i in range(40):
        t = i * 0.1
        monitor.update(t, state(arm=i * 0.05))
    # The arm has stopped; only the gripper is still travelling, in jumps far
    # bigger than motion_eps.
    monitor.update(t + 0.1, state(arm=39 * 0.05, left_finger=0.040))
    monitor.update(t + 0.2, state(arm=39 * 0.05, left_finger=0.020))
    verdict = monitor.update(t + 1.0, state(arm=39 * 0.05, left_finger=0.002))
    assert verdict.done and verdict.ok
