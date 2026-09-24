"""Ending a phase on something other than proprioception.

The grasp and settle conditions read ``/joint_states``, so they can only answer
questions about the robot. These two answer questions about the world: is the
gripper actually loaded, and did the scanner read the item.
"""

import pytest
import yaml

from cerebel_orchestrator.mission import Mission, MissionError, Signal, Until
from cerebel_orchestrator.phase_monitor import MonitorConfig, PhaseMonitor
from cerebel_orchestrator.signal_source import interpret

CONFIG = MonitorConfig(grasp_close_m=0.010, grasp_open_m=0.030, settle_grace_s=0.0)
OPEN = 0.040
SHUT = 0.005


def state(left_finger=OPEN, right_finger=OPEN):
    """A 16-D canonical joint vector with the two fingers set."""
    values = [0.0] * 16
    values[7] = left_finger
    values[15] = right_finger
    return values


def efforts(left=0.0, right=0.0):
    values = [0.0] * 16
    values[7] = left
    values[15] = right
    return values


def monitor(until: Until, start=0.0) -> PhaseMonitor:
    return PhaseMonitor(until, CONFIG, start)


# -- effort ------------------------------------------------------------------


def test_effort_alone_ends_a_phase_when_the_gripper_loads_up():
    mon = monitor(Until(timeout_s=60, effort=0.5, side="left", hold_s=0.5))

    mon.observe(0.0, state())
    mon.observe_effort(efforts(left=0.05))          # idle, nothing held
    assert not mon.verdict(0.0).done

    mon.observe(1.0, state(left_finger=SHUT))
    mon.observe_effort(efforts(left=2.4))           # gripping
    assert not mon.verdict(1.0).done, "must be held for hold_s"

    mon.observe(1.6, state(left_finger=SHUT))
    mon.observe_effort(efforts(left=2.4))
    verdict = mon.verdict(1.6)
    assert verdict.done and verdict.ok
    assert "loaded" in verdict.reason


def test_effort_needs_no_arming_because_an_idle_gripper_reads_zero():
    mon = monitor(Until(timeout_s=60, effort=0.5, side="left", hold_s=0.0))
    mon.observe(0.0, state())
    mon.observe_effort(efforts(left=3.0))
    assert mon.verdict(0.0).done, "a loaded gripper at t=0 is still a loaded gripper"


def test_the_effort_magnitude_is_taken_so_sign_conventions_do_not_matter():
    mon = monitor(Until(timeout_s=60, effort=0.5, side="left", hold_s=0.0))
    mon.observe(0.0, state())
    mon.observe_effort(efforts(left=-3.0))
    assert mon.verdict(0.0).done


def test_effort_dropping_below_the_threshold_resets_the_hold():
    """A grip that slips has not been held; the timer starts again."""
    mon = monitor(Until(timeout_s=60, effort=1.0, side="left", hold_s=0.5))
    mon.observe(0.0, state())
    mon.observe_effort(efforts(left=2.0))
    mon.observe(0.3, state())
    mon.observe_effort(efforts(left=0.1))           # slipped
    assert not mon.verdict(0.3).done
    mon.observe(0.6, state())
    mon.observe_effort(efforts(left=2.0))
    assert not mon.verdict(0.6).done, "the hold restarts from the slip"
    mon.observe(1.2, state())
    mon.observe_effort(efforts(left=2.0))
    assert mon.verdict(1.2).done


def test_a_driver_with_no_effort_field_times_out_and_says_so():
    """An empty effort array is a driver property, not a mission error -- but
    the mission must not look like it is nearly finished."""
    mon = monitor(Until(timeout_s=5, effort=0.5, side="left"))
    # Keep proprioception alive to the timeout, or the staleness check -- which
    # is a different failure -- fires first.
    for tick in range(60):
        mon.observe(tick * 0.1, state())
        mon.observe_effort([])
    verdict = mon.verdict(6.0)
    assert verdict.done and not verdict.ok
    assert "no effort field" in verdict.reason


def test_effort_and_grasp_together_must_both_hold():
    """Position says the gripper closed; effort says it closed on something."""
    until = Until(timeout_s=60, grasp="closed", side="left", effort=1.0, hold_s=0.0)
    mon = monitor(until)

    mon.observe(0.0, state(left_finger=OPEN))       # arms the grasp
    mon.observe_effort(efforts(left=0.0))
    assert not mon.verdict(0.0).done

    # Closed on air: position says yes, effort says no.
    mon.observe(1.0, state(left_finger=SHUT))
    mon.observe_effort(efforts(left=0.02))
    assert not mon.verdict(1.0).done, "a gripper closed on nothing is not a grasp"

    mon.observe(2.0, state(left_finger=SHUT))
    mon.observe_effort(efforts(left=2.5))
    assert mon.verdict(2.0).done


# -- external signals --------------------------------------------------------


def test_an_event_signal_latches_so_a_brief_firing_is_not_missed():
    """A barcode read is instantaneous and the verdict runs at 10 Hz."""
    mon = monitor(Until(timeout_s=60, signal="scanned"))
    mon.arm_signal_now()

    mon.observe_signal(True, "4006381333931")       # the scan
    mon.observe_signal(False)                       # and it is gone again

    verdict = mon.verdict(1.0)
    assert verdict.done and verdict.ok
    assert "4006381333931" in verdict.reason, "the code belongs in the run log"


def test_a_level_signal_must_be_seen_false_before_it_may_fire():
    """A box that already has something in it must not satisfy 'something
    arrived in the box' on the first tick."""
    mon = monitor(Until(timeout_s=60, signal="box_occupied"))

    mon.observe_signal(True)                        # true from the start
    assert not mon.verdict(1.0).done

    mon.observe_signal(False)                       # arms
    mon.observe_signal(True)                        # now it counts
    assert mon.verdict(2.0).done


def test_a_never_armed_level_signal_times_out_by_name():
    mon = monitor(Until(timeout_s=5, signal="box_occupied"))
    mon.observe_signal(True)
    verdict = mon.verdict(6.0)  # no joint_states at all: the timeout is the end
    assert verdict.done and not verdict.ok
    assert "never armed" in verdict.reason and "box_occupied" in verdict.reason


def test_a_signal_that_never_fires_reports_its_last_reading():
    mon = monitor(Until(timeout_s=5, signal="scanned"))
    mon.arm_signal_now()
    mon.observe_signal(False)
    verdict = mon.verdict(6.0)
    assert "never fired" in verdict.reason and "false" in verdict.reason


def test_a_signal_is_not_a_shortcut_past_the_other_conditions():
    """The scanner reading the item does not mean the item is in the box."""
    until = Until(
        timeout_s=60, signal="scanned", grasp="open", side="right", hold_s=0.0
    )
    mon = monitor(until)
    mon.arm_signal_now()

    mon.observe(0.0, state(right_finger=SHUT))      # arms the grasp: handover
    mon.observe_signal(True, "4006381333931")
    assert not mon.verdict(0.0).done, "scanned, but still holding it"

    mon.observe(1.0, state(right_finger=OPEN))      # released into the box
    assert mon.verdict(1.0).done


# -- reading a sensor message ------------------------------------------------


class Message:
    def __init__(self, data):
        self.data = data


def signal(**kwargs) -> Signal:
    body = {"topic": "/t"}
    body.update(kwargs)
    return Signal.parse("s", body)


def test_any_non_empty_string_is_a_firing_and_the_text_is_kept():
    scanner = signal(type="string")
    assert interpret(scanner, Message("4006381333931")) == (True, "4006381333931")
    assert interpret(scanner, Message("  ")) == (False, "")
    assert interpret(scanner, Message("")) == (False, "")


def test_a_bool_signal_is_read_as_it_stands():
    beam = signal(type="bool")
    assert interpret(beam, Message(True))[0] is True
    assert interpret(beam, Message(False))[0] is False


def test_a_float_signal_is_compared_against_its_threshold():
    cell = signal(type="float", above=0.05)
    assert interpret(cell, Message(0.12))[0] is True
    assert interpret(cell, Message(0.01))[0] is False


def test_a_float_band_rejects_a_reading_that_is_too_large():
    """The weight arrived, and it is not the whole box."""
    cell = signal(type="float", above=0.05, below=0.50)
    assert interpret(cell, Message(0.20))[0] is True
    assert interpret(cell, Message(2.00))[0] is False


def test_an_unreadable_message_is_none_rather_than_a_crash():
    assert interpret(signal(type="float", above=1.0), Message("not a number")) is None
    assert interpret(signal(type="bool"), object()) is None


# -- the mission schema ------------------------------------------------------

MISSION = """
name: s
signals:
  scanned: {topic: /barcode/code, type: string, mode: event}
policies:
  place:
    task_description: "scan and place the object"
    server_host: 127.0.0.1
    server_port: 5555
steps:
  - step: run_policy
    policy: place
    until: {timeout_s: 150, signal: scanned, grasp: open, side: right, settled: true}
"""


def test_a_signal_condition_round_trips():
    mission = Mission.from_dict(yaml.safe_load(MISSION))
    assert mission.signals["scanned"].topic == "/barcode/code"
    assert mission.signals["scanned"].mode == "event"
    assert mission.steps[0].until.signal == "scanned"


def test_an_undeclared_signal_is_rejected_by_name():
    with pytest.raises(MissionError, match="beam"):
        Mission.from_dict(yaml.safe_load(MISSION.replace("signal: scanned", "signal: beam")))


def test_a_float_signal_without_a_threshold_is_rejected():
    with pytest.raises(MissionError, match="above"):
        signal(type="float")


def test_thresholds_on_a_non_float_signal_are_rejected():
    with pytest.raises(MissionError, match="float"):
        signal(type="string", above=1.0)


def test_effort_without_a_side_is_rejected():
    with pytest.raises(MissionError, match="side"):
        Until.parse({"timeout_s": 60, "effort": 1.0}, "until")


def test_a_negative_effort_threshold_is_rejected():
    with pytest.raises(MissionError, match="magnitude"):
        Until.parse({"timeout_s": 60, "effort": -1.0, "side": "left"}, "until")


def test_a_sensor_condition_means_the_timeout_is_a_failure():
    """With no early condition, running the clock out is the intended end. With
    one, it is a failure -- and that must hold for the new conditions too."""
    assert Until.parse({"timeout_s": 60}, "u").timeout_is_success
    assert not Until.parse({"timeout_s": 60, "signal": "s"}, "u").timeout_is_success
    assert not Until.parse(
        {"timeout_s": 60, "effort": 1.0, "side": "left"}, "u"
    ).timeout_is_success
