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
