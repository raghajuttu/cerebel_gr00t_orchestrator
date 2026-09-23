"""Preflight: proving the policy server answers before the robot commits.

The ping itself needs pyzmq and a server, so these tests cover everything around
it — address parsing, the one-ping-per-server rule, and how failures are reported
— with the ping stubbed out.
"""

import pytest
import yaml

from cerebel_orchestrator import policy_preflight
from cerebel_orchestrator.mission import Mission
from cerebel_orchestrator.policy_preflight import (
    PingResult,
    failures,
    parse_address,
    preflight,
    summary,
)

MISSION = """
name: t
policies:
  pick_cube:
    task_description: "pick up the green cube and place it in the box"
    server_host: 127.0.0.1
    server_port: 5555
  place_cube:
    task_description: "pick up the green cube and place it in the box"
    server_host: 127.0.0.1
    server_port: 5555
steps:
  - {step: run_policy, policy: pick_cube, until: {timeout_s: 30}}
"""


@pytest.fixture
def pings(monkeypatch):
    """Record every ping and return whatever the test queued up."""
    calls = []
    outcomes = {}

    def fake_ping(host, port, timeout_ms=5000):
        calls.append((host, port, timeout_ms))
        ok, detail = outcomes.get(f"{host}:{port}", (True, "stub"))
        return PingResult(host, port, ok, detail, 0.01)

    monkeypatch.setattr(policy_preflight, "ping_server", fake_ping)
    return calls, outcomes


def test_addresses_parse_the_ways_people_type_them():
    assert parse_address("127.0.0.1:5555") == ("127.0.0.1", 5555)
    assert parse_address("cthor:5556") == ("cthor", 5556)
    assert parse_address("5555") == ("127.0.0.1", 5555)
    assert parse_address("cthor") == ("cthor", 5555)
    assert parse_address(" 127.0.0.1:5555 ") == ("127.0.0.1", 5555)
    with pytest.raises(ValueError):
        parse_address("")


def test_one_checkpoint_two_prompts_is_pinged_once(pings):
    calls, _ = pings
    mission = Mission.from_dict(yaml.safe_load(MISSION))
    results = preflight(mission.policy_ports(), timeout_ms=1000)
    # Two policies, one server: pinging it twice would prove nothing extra.
    assert len(calls) == 1
    assert calls[0] == ("127.0.0.1", 5555, 1000)
    assert len(results) == 1 and results[0].ok


def test_two_checkpoints_are_pinged_separately(pings):
    calls, _ = pings
    raw = yaml.safe_load(MISSION)
    raw["policies"]["place_cube"]["server_port"] = 5556
    preflight(Mission.from_dict(raw).policy_ports(), timeout_ms=1000)
    assert sorted(port for _, port, _ in calls) == [5555, 5556]


def test_a_dead_server_is_reported_with_its_address_and_reason(pings):
    _, outcomes = pings
    outcomes["127.0.0.1:5555"] = (False, "no reply within 1000 ms")
    mission = Mission.from_dict(yaml.safe_load(MISSION))
    results = preflight(mission.policy_ports(), timeout_ms=1000)
    assert failures(results)
    text = summary(results)
    assert "127.0.0.1:5555" in text and "no reply" in text


def test_one_dead_server_among_several_is_named(pings):
    _, outcomes = pings
    outcomes["127.0.0.1:5556"] = (False, "connection refused")
    raw = yaml.safe_load(MISSION)
    raw["policies"]["place_cube"]["server_port"] = 5556
    results = preflight(Mission.from_dict(raw).policy_ports(), timeout_ms=1000)
    assert len(failures(results)) == 1
    assert "5556" in summary(results) and "5555" not in summary(results)


def test_all_reachable_says_so(pings):
    mission = Mission.from_dict(yaml.safe_load(MISSION))
    results = preflight(mission.policy_ports(), timeout_ms=1000)
    assert failures(results) == []
    assert summary(results) == "all 1 policy server(s) reachable"


def test_the_logger_gets_a_line_per_server(pings):
    _, outcomes = pings
    outcomes["127.0.0.1:5555"] = (False, "no reply")

    class Recorder:
        def __init__(self):
            self.info, self.error = [], []

        def __getattr__(self, name):
            raise AttributeError(name)

    recorder = Recorder()
    logged = {"info": [], "error": []}
    recorder.info = logged["info"].append
    recorder.error = logged["error"].append

    mission = Mission.from_dict(yaml.safe_load(MISSION))
    preflight(mission.policy_ports(), timeout_ms=1000, logger=recorder)
    assert logged["error"] and not logged["info"]
    # The line names the policies that would have used that server.
    assert "pick_cube" in logged["error"][0] and "place_cube" in logged["error"][0]


def test_a_missing_pyzmq_is_a_failure_not_a_crash(monkeypatch):
    """The check must never be the thing that breaks the run."""
    import builtins

    real_import = builtins.__import__

    def no_zmq(name, *args, **kwargs):
        if name in ("zmq", "msgpack", "msgpack_numpy"):
            raise ImportError(f"No module named {name!r}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_zmq)
    result = policy_preflight.ping_server("127.0.0.1", 5555, timeout_ms=10)
    assert not result.ok and "pyzmq" in result.detail


def test_describe_is_readable():
    ok = PingResult("127.0.0.1", 5555, True, "server replied 'pong'", 0.012)
    assert ok.describe().startswith("OK   127.0.0.1:5555 (12 ms)")
    bad = PingResult("127.0.0.1", 5555, False, "no reply", 5.0)
    assert bad.describe().startswith("FAIL 127.0.0.1:5555")
