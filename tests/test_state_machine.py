"""Mission control flow: order, retries, repeats, and what a hold does."""

import yaml

from cerebel_orchestrator.mission import Mission
from cerebel_orchestrator.state_machine import MissionRunner, Phase

BASE = """
name: t
stations:
  pick:  {x: 0.0, y: 0.4, yaw: 0.0}
  place: {x: 0.0, y: -0.4, yaw: 0.0}
policies:
  grab:
    task_description: "pick up the green cube and place it in the box"
    server_host: 127.0.0.1
    server_port: 5555
steps:
  - {step: navigate, station: pick}
  - {step: run_policy, policy: grab, until: {timeout_s: 30}}
  - {step: navigate, station: place}
"""


def runner(text=BASE, **overrides):
    raw = yaml.safe_load(text)
    raw.update(overrides)
    run = MissionRunner(Mission.from_dict(raw))
    run.start()
    return run


def drive(run, outcomes):
    """Report a sequence of (ok, reason) outcomes, returning the actions seen."""
    seen = []
    for ok, reason in outcomes:
        action = run.pending()
        assert action is not None, f"expected an action, phase is {run.phase}"
        seen.append(action)
        run.report(ok, reason)
    return seen


def test_steps_run_in_order_then_the_mission_is_done():
    run = runner()
    seen = drive(run, [(True, "")] * 3)
    assert [a.kind for a in seen] == ["navigate", "run_policy", "navigate"]
    assert seen[0].station.y == 0.4
    assert seen[1].policy.server_port == 5555
    assert run.phase is Phase.DONE
    assert run.terminal and run.pending() is None


def test_phase_tracks_the_current_step():
    run = runner()
    assert run.phase is Phase.NAVIGATE
    run.pending()
    run.report(True)
    assert run.phase is Phase.POLICY


def test_repeat_runs_the_whole_list_again():
    run = runner(repeat=2)
    seen = drive(run, [(True, "")] * 6)
    assert [a.cycle for a in seen] == [1, 1, 1, 2, 2, 2]
    assert run.phase is Phase.DONE


def test_failure_aborts_by_default_and_names_the_step():
    run = runner()
    run.pending()
    run.report(False, "nav2 rejected the goal")
    assert run.phase is Phase.ABORTED
    assert "step 0" in run.reason and "nav2 rejected the goal" in run.reason
    assert run.pending() is None


def test_retry_reissues_the_same_step_then_gives_up():
    text = BASE.replace(
        "{step: navigate, station: pick}",
        "{step: navigate, station: pick, on_fail: retry, retries: 2}",
    )
    run = runner(text)
    attempts = []
    for _ in range(3):
        action = run.pending()
        attempts.append(action.attempt)
        assert action.step.index == 0
        run.report(False, "no path")
    assert attempts == [0, 1, 2]
    assert run.phase is Phase.ABORTED
    assert "3x" in run.reason


def test_a_successful_retry_resets_the_counter():
    text = BASE.replace(
        "{step: navigate, station: pick}",
        "{step: navigate, station: pick, on_fail: retry, retries: 2}",
    )
    run = runner(text)
    run.pending()
    run.report(False, "no path")
    action = run.pending()
    assert action.attempt == 1
    run.report(True)
    assert run.phase is Phase.POLICY
    assert run.attempt == 0


def test_continue_moves_past_a_failed_step():
    text = BASE.replace(
        "until: {timeout_s: 30}}", "until: {timeout_s: 30}, on_fail: continue}"
    )
    run = runner(text)
    drive(run, [(True, ""), (False, "gripper never closed"), (True, "")])
    assert run.phase is Phase.DONE
    assert run.status()["failed"] == 1


def test_hold_restarts_the_step_without_spending_a_retry():
    text = BASE.replace(
        "{step: navigate, station: pick}",
        "{step: navigate, station: pick, on_fail: retry, retries: 1}",
    )
    run = runner(text)
    first = run.pending()
    run.hold("e-stop")
    assert run.phase is Phase.HOLD and run.pending() is None
    run.resume()
    again = run.pending()
    assert run.phase is Phase.NAVIGATE
    assert again.step.index == first.step.index
    assert again.attempt == 0


def test_hold_mid_policy_returns_to_the_policy_phase():
    run = runner()
    run.pending()
    run.report(True)
    run.pending()
    run.hold("e-stop")
    run.resume()
    assert run.phase is Phase.POLICY
    assert run.pending().kind == "run_policy"


def test_abort_and_fault_are_terminal_and_keep_their_reason():
    run = runner()
    run.abort("operator pressed stop")
    assert run.phase is Phase.ABORTED and run.reason == "operator pressed stop"
    run.resume()
    assert run.phase is Phase.ABORTED

    other = runner()
    other.fault("nav2 action server never appeared")
    assert other.phase is Phase.FAULT
    other.abort("too late")
    assert other.phase is Phase.FAULT


def test_run_label_identifies_the_attempt_for_the_run_log():
    text = BASE.replace(
        "until: {timeout_s: 30}}", "until: {timeout_s: 30}, on_fail: retry, retries: 1}"
    )
    run = runner(text)
    run.pending()
    run.report(True)
    assert run.pending().run_label == "t_c1_s1_run_policy_grab"
    run.report(False, "x")
    assert run.pending().run_label == "t_c1_s1_run_policy_grab_a2"


def test_status_and_summary_describe_the_run():
    run = runner()
    drive(run, [(True, ""), (False, "timed out")])
    status = run.status()
    assert status["phase"] == "aborted"
    assert status["completed"] == 1 and status["failed"] == 1
    assert "FAIL" in run.summary() and "timed out" in run.summary()
