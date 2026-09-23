"""Joint envelopes: the measurement that backs up an `ends_parked` claim."""

import pytest
import yaml

from cerebel_orchestrator.envelopes import Envelope, EnvelopeError, load_envelopes
from cerebel_orchestrator.joints import GRIPPER_INDEX

CARRY = """
envelopes:
  carry:
    description: holding the object, tucked enough to drive
    joints:
      openarm_left_joint2: [-1.60, -0.80]
      openarm_left_joint4: [-2.20, -1.20]
      openarm_left_finger_joint1: [-0.01, 0.02]
"""


def envelope():
    return {
        name: Envelope.parse(name, body)
        for name, body in yaml.safe_load(CARRY)["envelopes"].items()
    }["carry"]


def pose(joint2=-1.2, joint4=-1.7, finger=0.005):
    positions = [0.0] * 16
    positions[1] = joint2
    positions[3] = joint4
    positions[GRIPPER_INDEX["left"]] = finger
    return positions


def test_a_pose_inside_every_bound_passes():
    assert envelope().check(pose()) is None


def test_an_unconstrained_joint_is_free():
    positions = pose()
    positions[5] = 3.0  # joint6 is not in the envelope
    assert envelope().check(positions) is None


def test_a_violation_names_the_joint_and_both_bounds():
    violation = envelope().check(pose(joint2=-0.2))
    assert violation is not None
    assert "openarm_left_joint2" in violation
    assert "-0.2000" in violation and "-1.6000" in violation and "-0.8000" in violation


def test_every_violation_is_reported_not_just_the_first():
    violation = envelope().check(pose(joint2=0.0, joint4=0.0))
    assert "openarm_left_joint2" in violation and "openarm_left_joint4" in violation


def test_a_dropped_object_is_caught_by_the_finger_bound():
    # The arm is exactly where it should be, but the gripper is open: the object
    # is on the floor and the base is about to drive off without it.
    violation = envelope().check(pose(finger=0.040))
    assert violation is not None and "finger" in violation


def test_the_wrong_number_of_joints_is_an_error():
    with pytest.raises(ValueError, match="16 canonical joints"):
        envelope().check([0.0] * 15)


def test_loading_rejects_nonsense():
    def parse(text):
        return Envelope.parse("e", yaml.safe_load(text))

    with pytest.raises(EnvelopeError, match="not a canonical joint name"):
        parse("joints: {elbow: [0, 1]}")
    with pytest.raises(EnvelopeError, match="min 1.0 is not below max"):
        parse("joints: {openarm_left_joint2: [1.0, 0.0]}")
    with pytest.raises(EnvelopeError, match=r"\[min, max\]"):
        parse("joints: {openarm_left_joint2: [0.0]}")
    with pytest.raises(EnvelopeError, match="constrains nothing"):
        parse("description: empty")
    with pytest.raises(EnvelopeError, match="unknown keys"):
        parse("jonits: {openarm_left_joint2: [0.0, 1.0]}")


def test_no_envelope_file_means_no_envelopes():
    assert load_envelopes("") == {}


def test_the_shipped_envelope_file_parses(tmp_path):
    import pathlib

    shipped = pathlib.Path(__file__).resolve().parent.parent / "params" / "arm_envelopes.yaml"
    envelopes = load_envelopes(str(shipped))
    assert {"carry", "home"} <= set(envelopes)
    # The carry envelope must constrain the gripper -- that bound is what
    # catches a dropped object before the base moves.
    assert "openarm_left_finger_joint1" in envelopes["carry"].limits
