"""Mission-file validation: the things that must not reach hardware."""

import pytest
import yaml

from cerebel_orchestrator.mission import Mission, MissionError

GOOD = """
name: t
frame_id: odom
stations:
  pick:  {x: 0.0, y: 0.45, yaw: 0.0}
  place: {x: 0.0, y: -0.45, yaw: 0.0}
policies:
  grab:
    task_description: "pick up the green cube and place it in the box"
    server_host: 127.0.0.1
    server_port: 5555
    params: {execution_horizon: 16}
steps:
  - {step: park_arms, profile: travel}
  - {step: navigate, station: pick}
  - {step: run_policy, policy: grab, until: {timeout_s: 60, grasp: closed, side: left}}
"""


def load(text):
    return Mission.from_dict(yaml.safe_load(text))


def test_good_mission_round_trips():
    mission = load(GOOD)
    assert mission.frame_id == "odom"
    assert [s.kind for s in mission.steps] == ["park_arms", "navigate", "run_policy"]
    assert mission.steps[2].until.grasp == "closed"
    assert mission.policies["grab"].params["execution_horizon"] == 16
    assert mission.stations["place"].y == pytest.approx(-0.45)


def test_unknown_station_is_rejected_by_name():
    text = GOOD.replace("station: pick", "station: nowhere")
    with pytest.raises(MissionError) as exc:
        load(text)
    assert "nowhere" in str(exc.value) and "steps[1]" in str(exc.value)


def test_unknown_policy_is_rejected():
    text = GOOD.replace("policy: grab", "policy: ghost")
    with pytest.raises(MissionError, match="ghost"):
        load(text)


def test_policy_phase_without_timeout_is_rejected():
    text = GOOD.replace("timeout_s: 60, ", "")
    with pytest.raises(MissionError, match="timeout_s"):
        load(text)


def test_grasp_without_side_is_rejected():
    text = GOOD.replace(", side: left", "")
    with pytest.raises(MissionError, match="side"):
        load(text)


def test_degrees_in_yaw_are_caught():
    text = GOOD.replace("yaw: 0.0}\n  place", "yaw: 90.0}\n  place")
    with pytest.raises(MissionError, match="radians"):
        load(text)


def test_params_may_not_shadow_the_policy_body():
    text = GOOD.replace(
        "params: {execution_horizon: 16}", 'params: {task_description: "sneaky"}'
    )
    with pytest.raises(MissionError, match="task_description"):
        load(text)


def test_retry_needs_a_count_and_a_count_needs_retry():
    with pytest.raises(MissionError, match="retries > 0"):
        load(GOOD.replace("station: pick}", "station: pick, on_fail: retry}"))
    with pytest.raises(MissionError, match="not 'retry'"):
        load(GOOD.replace("station: pick}", "station: pick, retries: 2}"))


def test_typo_in_a_step_key_is_not_silently_ignored():
    with pytest.raises(MissionError, match="unknown keys"):
        load(GOOD.replace("station: pick}", "statoin: pick}"))


def test_shared_port_is_reported_as_one_checkpoint():
    raw = yaml.safe_load(GOOD)
    raw["policies"]["drop"] = {
        "task_description": "place the green cube in the box",
        "server_host": "127.0.0.1",
        "server_port": 5555,
    }
    raw["policies"]["stage"] = {
        "task_description": "hand the cube over",
        "server_host": "127.0.0.1",
        "server_port": 5556,
    }
    ports = Mission.from_dict(raw).policy_ports()
    assert set(ports["127.0.0.1:5555"]) == {"grab", "drop"}
    assert ports["127.0.0.1:5556"] == ["stage"]


def test_timeout_only_phase_counts_the_timeout_as_success():
    text = GOOD.replace("timeout_s: 60, grasp: closed, side: left", "timeout_s: 60")
    assert load(text).steps[2].until.timeout_is_success
    assert not load(GOOD).steps[2].until.timeout_is_success


# -- the park-before-navigate lint -----------------------------------------


def lint(steps, repeat=1):
    from cerebel_orchestrator.mission import navigate_safety_warnings

    raw = yaml.safe_load(GOOD)
    raw["steps"] = steps
    raw["repeat"] = repeat
    return navigate_safety_warnings(Mission.from_dict(raw))


PARK = {"step": "park_arms", "profile": "travel"}
NAV = {"step": "navigate", "station": "pick"}
WAIT = {"step": "wait", "seconds": 1.0}
POLICY = {"step": "run_policy", "policy": "grab", "until": {"timeout_s": 10}}


def test_a_mission_without_navigation_is_never_linted():
    assert lint([POLICY]) == []


def test_park_then_navigate_is_clean():
    assert lint([PARK, NAV]) == []


def test_a_wait_between_park_and_navigate_is_still_clean():
    assert lint([PARK, NAV, WAIT, NAV]) == []


def test_a_policy_between_park_and_navigate_is_flagged():
    warnings = lint([PARK, NAV, POLICY, NAV])
    assert len(warnings) == 1
    assert "steps[3]" in warnings[0] and "run_policy" in warnings[0]


def test_navigating_before_any_park_is_flagged_once():
    warnings = lint([NAV, PARK, NAV])
    assert len(warnings) == 1
    assert "steps[0]" in warnings[0] and "already parked" in warnings[0]


def test_never_parking_at_all_is_one_blunt_warning():
    warnings = lint([NAV, POLICY, NAV])
    assert len(warnings) == 1
    assert "never parks" in warnings[0]


def test_a_repeating_mission_is_linted_across_the_wrap():
    # One pass only notices that step 0 drives before anything parked. Repeating
    # it also means cycle 2 arrives at step 0 straight out of the policy phase,
    # which is a second, different problem.
    steps = [NAV, PARK, POLICY]
    once = lint(steps, repeat=1)
    assert len(once) == 1 and "already parked" in once[0]

    twice = lint(steps, repeat=2)
    assert len(twice) == 2
    assert any("already parked" in w for w in twice)
    assert any("run_policy" in w and "steps[0]" in w for w in twice)


def test_a_repeating_mission_that_parks_last_is_clean():
    assert lint([PARK, NAV, POLICY, PARK], repeat=3) == []


# -- ends_parked and check_arms ---------------------------------------------


CARRY_STEPS = [
    PARK,
    NAV,
    {
        "step": "run_policy",
        "policy": "grab",
        "until": {"timeout_s": 90, "grasp": "closed", "side": "left", "settled": True},
        "ends_parked": True,
    },
    {"step": "check_arms", "envelope": "carry", "seconds": 5.0},
    NAV,
]


def test_a_policy_that_ends_parked_satisfies_the_lint():
    assert lint(CARRY_STEPS) == []


def test_ends_parked_without_settled_is_refused():
    raw = yaml.safe_load(GOOD)
    raw["steps"] = [
        {
            "step": "run_policy",
            "policy": "grab",
            "until": {"timeout_s": 90, "grasp": "closed", "side": "left"},
            "ends_parked": True,
        }
    ]
    with pytest.raises(MissionError, match="no `settled`"):
        Mission.from_dict(raw)


def test_check_arms_also_satisfies_the_lint_on_its_own():
    steps = [
        PARK,
        NAV,
        {"step": "run_policy", "policy": "grab", "until": {"timeout_s": 10}},
        {"step": "check_arms", "envelope": "carry"},
        NAV,
    ]
    assert lint(steps) == []


def test_check_arms_needs_an_envelope_name():
    raw = yaml.safe_load(GOOD)
    raw["steps"] = [{"step": "check_arms"}]
    with pytest.raises(MissionError, match="envelope"):
        Mission.from_dict(raw)


def test_a_mission_with_only_checks_is_not_told_it_never_parks():
    steps = [
        {"step": "check_arms", "envelope": "home"},
        NAV,
    ]
    assert lint(steps) == []


def test_operator_cannot_be_mixed_with_a_watched_condition():
    raw = yaml.safe_load(GOOD)
    raw["steps"] = [
        {
            "step": "run_policy",
            "policy": "grab",
            "until": {"timeout_s": 30, "operator": True, "settled": True},
        }
    ]
    with pytest.raises(MissionError, match="operator cannot be combined"):
        Mission.from_dict(raw)


def test_the_shipped_pick_and_carry_mission_is_clean():
    import pathlib

    from cerebel_orchestrator.mission import navigate_safety_warnings

    root = pathlib.Path(__file__).resolve().parent.parent
    mission = Mission.load(str(root / "missions" / "two_station_pick_place.yaml"))
    assert navigate_safety_warnings(mission) == []
    # The drive to the place station happens with the object in hand: no park
    # between the pick policy and that navigate.
    kinds = [step.kind for step in mission.steps]
    pick = kinds.index("run_policy")
    assert kinds[pick + 1] == "check_arms" and kinds[pick + 2] == "navigate"
    assert mission.steps[pick].ends_parked
    assert mission.steps[pick].until.settled and mission.steps[pick].until.grasp == "closed"


# -- move_base: positions, the axis anchor, and the lint ---------------------

MOVES = """
name: m
positions:
  p1: {axis_cm: 0.0}
  p2: {axis_cm: 38.0}
start_position: p1
policies:
  grab:
    task_description: "pick up lipstick"
    server_host: 127.0.0.1
    server_port: 5555
steps:
  - {step: park_arms, profile: travel}
  - {step: move_base, to: p2}
  - {step: run_policy, policy: grab, until: {timeout_s: 60, grasp: closed, side: left}}
"""


def test_move_base_mission_round_trips():
    mission = load(MOVES)
    assert mission.start_position == "p1"
    assert mission.positions["p2"].axis_cm == pytest.approx(38.0)
    assert mission.steps[1].kind == "move_base"
    assert mission.steps[1].position == "p2"
    assert mission.steps[1].axis == "lateral", "lateral is the default axis"


def test_unknown_position_is_rejected_by_name():
    with pytest.raises(MissionError, match="p9"):
        load(MOVES.replace("to: p2", "to: p9"))


def test_move_base_without_a_start_position_is_rejected():
    """Every move distance is measured from the anchor, so there must be one."""
    with pytest.raises(MissionError, match="start_position"):
        load(MOVES.replace("start_position: p1\n", ""))


def test_start_position_must_name_a_real_position():
    with pytest.raises(MissionError, match="start_position"):
        load(MOVES.replace("start_position: p1", "start_position: nowhere"))


def test_axis_must_be_one_of_the_two():
    with pytest.raises(MissionError, match="axis"):
        load(MOVES.replace("{step: move_base, to: p2}", "{step: move_base, to: p2, axis: up}"))


def test_axis_cm_that_looks_like_metres_is_rejected():
    """0.38 would be a silent 38x error; the range check is what catches it."""
    with pytest.raises(MissionError, match="centimetres"):
        load(MOVES.replace("axis_cm: 38.0", "axis_cm: 5000.0"))


def test_a_navigate_station_is_not_usable_as_a_move_base_position():
    """The two tables are separate on purpose -- one is a pose, one is not."""
    with pytest.raises(MissionError, match="not in positions"):
        load(
            MOVES.replace("positions:\n  p1: {axis_cm: 0.0}\n  p2: {axis_cm: 38.0}",
                          "positions:\n  p1: {axis_cm: 0.0}\nstations:\n  p2: {x: 0.0, y: 0.4}")
        )


def test_the_lint_treats_move_base_like_navigate():
    """Driving with the arms wherever a policy left them is the hazard, and it
    does not care which stack is turning the wheels."""
    from cerebel_orchestrator.mission import navigate_safety_warnings

    # A park at the start, then a policy, then a move: the park is real, so
    # this exercises the walk-back rather than the "never parks at all" case.
    unsafe = MOVES.replace(
        "  - {step: move_base, to: p2}",
        "  - {step: run_policy, policy: grab, until: {timeout_s: 60}}\n"
        "  - {step: move_base, to: p2}",
    )
    warnings = navigate_safety_warnings(load(unsafe))
    assert warnings and any("move_base" in w for w in warnings), warnings


def test_the_lint_accepts_a_carry_move_after_an_ends_parked_policy():
    """Pick-and-carry: parking between the pick and the drive would drop the
    object, so `ends_parked` is the policy's claim that it finished drive-safe."""
    from cerebel_orchestrator.mission import navigate_safety_warnings

    carry = MOVES.replace(
        "  - {step: move_base, to: p2}",
        "  - {step: run_policy, policy: grab, ends_parked: true, "
        "until: {timeout_s: 60, grasp: closed, side: left, settled: true}}\n"
        "  - {step: move_base, to: p2}",
    )
    assert navigate_safety_warnings(load(carry)) == []


def test_the_lint_is_quiet_when_a_park_precedes_the_move():
    from cerebel_orchestrator.mission import navigate_safety_warnings

    assert navigate_safety_warnings(load(MOVES)) == []
