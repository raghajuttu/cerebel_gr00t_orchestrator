"""Holding the arms across a policy switch.

The case that matters: the pick ends with the object in the gripper, the client
is killed, and the place policy's client takes seconds to start. Nothing may
move the arms in between, and nothing may command them to the *measured*
position -- the follower sits a standing offset behind the command, so that
would walk the arm backwards at every switch.
"""

import pytest

from cerebel_orchestrator.arm_hold import ArmHold

PICK_POSE = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
RIGHT_POSE = [0.0, 0.1, 0.0, -0.1, 0.0, 0.1, 0.0]
QUIET = 0.25


def holder() -> ArmHold:
    return ArmHold(foreign_quiet_s=QUIET)


# -- nothing to hold ---------------------------------------------------------


def test_nothing_is_published_before_any_command_has_been_seen():
    hold = holder()
    assert not hold.ready
    assert hold.tick(now=10.0, allowed=True) == []


def test_nothing_is_published_while_something_else_owns_the_arms():
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    assert hold.tick(now=5.0, allowed=False) == []
    assert not hold.holding


# -- the switch --------------------------------------------------------------


def test_the_last_commanded_pose_is_republished_once_the_topic_goes_quiet():
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)

    # The client is still publishing, or between two of its own ticks.
    assert hold.tick(now=0.1, allowed=True) == []

    # The client is gone.
    out = hold.tick(now=1.0, allowed=True)
    assert out == [("left", PICK_POSE)]
    assert hold.holding


def test_both_arms_are_held_when_both_have_been_commanded():
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    hold.observe("right", RIGHT_POSE, now=0.0)
    out = dict(hold.tick(now=1.0, allowed=True))
    assert out == {"left": PICK_POSE, "right": RIGHT_POSE}


def test_the_held_pose_is_the_commanded_one_not_a_later_measurement():
    """There is no path by which a measured position can become the hold: the
    only input is the command topic."""
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    hold.tick(now=1.0, allowed=True)
    assert hold.pose()["left"] == PICK_POSE


def test_holding_survives_indefinitely():
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    for tick in range(1, 400):
        out = hold.tick(now=1.0 + tick * 0.033, allowed=True)
        assert out == [("left", PICK_POSE)]


# -- standing down -----------------------------------------------------------


def test_a_new_client_publishing_stands_the_hold_down():
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    assert hold.tick(now=1.0, allowed=True)

    # The place policy's first chunk lands.
    moved = [v + 0.05 for v in PICK_POSE]
    hold.observe("left", moved, now=1.1)
    assert not hold.holding
    assert hold.tick(now=1.2, allowed=True) == [], "must not compete with the client"

    # And the pose it would fall back to is now the client's, not the old one.
    assert hold.pose()["left"] == moved


def test_its_own_echo_does_not_count_as_somebody_else():
    """The node subscribes to the topic it publishes on. Without this, a hold
    would see its own message, decide a commander was present, and stand down
    -- then resume, forever, at the quiet interval."""
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    out = hold.tick(now=1.0, allowed=True)
    assert out == [("left", PICK_POSE)]

    hold.observe("left", PICK_POSE, now=1.001)      # the echo
    assert hold.holding, "an echo must not stand the hold down"
    assert hold.tick(now=1.033, allowed=True) == [("left", PICK_POSE)]


def test_a_park_ramp_takes_the_arms_back():
    """A park is deliberately moving the arms somewhere else, so the hold
    stands down outright rather than waiting for the quiet window."""
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    assert hold.tick(now=1.0, allowed=True)

    assert hold.tick(now=1.1, allowed=False) == []
    assert not hold.holding

    ramp = [v * 0.5 for v in PICK_POSE]
    hold.observe("left", ramp, now=1.2)
    assert hold.pose()["left"] == ramp, "the ramp's pose is what gets held after"


def test_forget_drops_the_pose_entirely():
    hold = holder()
    hold.observe("left", PICK_POSE, now=0.0)
    hold.forget()
    assert not hold.ready
    assert hold.tick(now=5.0, allowed=True) == []


# -- malformed input ---------------------------------------------------------


def test_an_empty_message_is_ignored():
    hold = holder()
    hold.observe("left", [], now=0.0)
    assert not hold.ready


def test_values_are_coerced_to_float():
    hold = holder()
    hold.observe("left", [1, 2, 3], now=0.0)
    out = hold.tick(now=1.0, allowed=True)
    assert out == [("left", [1.0, 2.0, 3.0])]
    assert all(isinstance(v, float) for v in out[0][1])


# -- staying out of a running policy's silences ------------------------------


def test_a_client_that_has_spoken_owns_the_rest_of_its_phase():
    """`inference_client` publishes NOTHING while it waits for the server -- in
    blocking mode that is the whole round trip. Those silences belong to the
    client; the controller latches through them."""
    hold = holder()
    hold.begin_phase()
    assert not hold.client_spoke

    hold.observe("left", PICK_POSE, now=0.0)        # the client's first chunk
    assert hold.client_spoke

    # The orchestrator inverts this into `allowed`, so a stall does not let the
    # hold in. Simulated here as the gate the node computes.
    allowed = not hold.client_spoke
    assert hold.tick(now=5.0, allowed=allowed) == []


def test_begin_phase_re_opens_the_hold_for_the_next_switch():
    hold = holder()
    hold.begin_phase()
    hold.observe("left", PICK_POSE, now=0.0)
    assert hold.client_spoke

    hold.begin_phase()                               # the next policy phase
    assert not hold.client_spoke
    assert hold.tick(now=5.0, allowed=True) == [("left", PICK_POSE)]


def test_the_echo_does_not_count_as_the_client_speaking():
    """Otherwise the hold's own first message would convince the node that the
    client had started, and the hold would switch itself off."""
    hold = holder()
    hold.begin_phase()
    hold.observe("left", PICK_POSE, now=0.0)         # client A, previous phase
    hold.begin_phase()
    hold.tick(now=5.0, allowed=True)                 # the hold publishes
    hold.observe("left", PICK_POSE, now=5.001)       # its own echo
    assert not hold.client_spoke, "an echo is not a client"
