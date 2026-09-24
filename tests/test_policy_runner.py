"""How a policy phase turns into a command line.

The command is the whole interface to the inference client, so it is worth
pinning down exactly: a task description that loses its spaces, or a float that
arrives as a string, is a phase that runs the wrong policy while looking healthy.
"""

import logging
import sys

import pytest
import yaml

from cerebel_orchestrator.mission import Mission
from cerebel_orchestrator.policy_runner import (
    PolicyPrepareError,
    PolicyRunner,
    RunnerConfig,
    describe_command,
    yaml_scalar,
)

MISSION = """
name: t
policies:
  grab:
    task_description: "pick up the green cube and place it in the box"
    server_host: 10.0.0.4
    server_port: 5556
    checkpoint_label: "v9.2-h40"
    params:
      execution_horizon: 16
      prefetch_enable: true
      rtc_enable: false
      enable_limits: false
      max_joint_speed: 0.25
steps:
  - {step: run_policy, policy: grab, until: {timeout_s: 30}}
"""


@pytest.fixture
def policy():
    return Mission.from_dict(yaml.safe_load(MISSION)).policies["grab"]


@pytest.fixture
def runner():
    return PolicyRunner(RunnerConfig(mock=True, log_dir="/tmp/x"), logging.getLogger("test"))


def params_of(argv):
    """The -p name:=value pairs, as a dict of raw strings."""
    out = {}
    for index, item in enumerate(argv):
        if item == "-p":
            name, _, value = argv[index + 1].partition(":=")
            out[name] = value
    return out


def test_scalars_survive_the_round_trip():
    assert yaml_scalar(True) == "true"
    assert yaml_scalar(False) == "false"
    assert yaml_scalar(16) == "16"
    assert yaml_scalar(0.25) == "0.25"
    assert yaml.safe_load(yaml_scalar("pick up the cube")) == "pick up the cube"
    # A string that looks like a number must stay a string.
    assert yaml.safe_load(yaml_scalar("5555")) == "5555"
    assert yaml.safe_load(yaml_scalar("it's here")) == "it's here"
    assert yaml.safe_load(yaml_scalar([1, 2])) == [1, 2]


def test_the_command_carries_the_policy_identity(runner, policy):
    argv = runner.build_argv(policy, "run_7")
    assert argv[:4] == ["ros2", "run", "adibot_gr00t_client", "inference_client"]
    assert "--ros-args" in argv
    params = params_of(argv)
    assert yaml.safe_load(params["task_description"]) == (
        "pick up the green cube and place it in the box"
    )
    assert yaml.safe_load(params["server_host"]) == "10.0.0.4"
    assert params["server_port"] == "5556"
    assert yaml.safe_load(params["log_run_name"]) == "run_7"
    assert yaml.safe_load(params["checkpoint_label"]) == "v9.2-h40"


def test_mission_params_reach_the_client_with_their_types(runner, policy):
    params = params_of(runner.build_argv(policy, "run_7"))
    assert params["execution_horizon"] == "16"
    assert params["prefetch_enable"] == "true"
    assert params["rtc_enable"] == "false"
    assert params["max_joint_speed"] == "0.25"


def test_the_client_command_is_configurable(policy):
    config = RunnerConfig(
        policy_cmd=["bash", "-lc", "source /opt/ros/humble/setup.bash && exec ros2 run pkg exe"],
        mock=True,
    )
    argv = PolicyRunner(config, logging.getLogger("test")).build_argv(policy, "r")
    assert argv[0] == "bash"
    assert "--ros-args" in argv


def test_mock_mode_starts_no_process_and_stops_cleanly(runner, policy):
    session = runner.start(policy, "run_1")
    assert session.mocked and runner.running
    assert runner.check() is None
    runner.stop("test over")
    assert not runner.running
    # Stopping twice is harmless -- the e-stop path can race the phase end.
    runner.stop("again")


def test_two_sessions_at_once_are_refused(runner, policy):
    runner.start(policy, "run_1")
    with pytest.raises(RuntimeError, match="still running"):
        runner.start(policy, "run_2")


def test_describe_command_matches_what_would_run(policy):
    config = RunnerConfig(mock=True)
    text = describe_command(policy, "LABEL", config)
    assert "inference_client" in text and "LABEL" in text
    assert "'pick up the green cube and place it in the box'" in text


# -- re-initialising the arm controller between policies ---------------------

def prepare_runner(cmd, **overrides):
    config = RunnerConfig(prepare_cmd=cmd, **overrides)
    return PolicyRunner(config, logging.getLogger("prepare-test"))


def test_no_prepare_command_is_a_no_op():
    assert prepare_runner([]).prepare() is None


def test_a_successful_prepare_returns_none():
    runner = prepare_runner([sys.executable, "-c", "pass"])
    assert runner.prepare() is None


def test_a_failing_prepare_reports_the_exit_code_and_the_tail():
    runner = prepare_runner(
        [sys.executable, "-c", "import sys; print('controller not found'); sys.exit(3)"]
    )
    error = runner.prepare()
    assert error is not None
    assert "exited 3" in error
    assert "controller not found" in error


def test_a_missing_prepare_command_is_named_not_raised():
    error = prepare_runner(["definitely-not-a-real-binary-xyzzy"]).prepare()
    assert error is not None and "not found" in error


def test_a_hanging_prepare_is_bounded_by_its_timeout():
    runner = prepare_runner(
        [sys.executable, "-c", "import time; time.sleep(30)"], prepare_timeout_s=0.5
    )
    error = runner.prepare()
    assert error is not None and "timed out" in error


def test_a_failed_prepare_stops_the_client_from_starting(policy):
    """The phase must fail before a process is handed the arms."""
    runner = prepare_runner([sys.executable, "-c", "import sys; sys.exit(1)"])
    with pytest.raises(PolicyPrepareError):
        runner.start(policy, "run")
    assert not runner.running, "no session may exist after a failed prepare"


def test_mock_skips_the_prepare_entirely(policy):
    runner = prepare_runner(
        ["definitely-not-a-real-binary-xyzzy"], mock=True, log_dir="/tmp/x"
    )
    assert runner.prepare() is None
    runner.start(policy, "run")
    assert runner.running
