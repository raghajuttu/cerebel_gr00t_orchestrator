"""policy_switching: prompt -- one client, its prompt switched per phase.

Three layers: the PromptClient bookkeeping (pure), the mission rule that only a
single-client mission may use it, and the node's gate -- the part that decides
whether a base move may start while a client could still be commanding the
arms. The gate lives on the ROS node, so ROS is stubbed when it is not
installed and the methods are called on a stand-in ``self``.
"""

import json
import sys
import types

import pytest

from cerebel_orchestrator.mission import Mission, MissionError
from cerebel_orchestrator.policy_runner import PolicyRunner, RunnerConfig
from cerebel_orchestrator.prompt_client import PromptClient


# -- PromptClient -------------------------------------------------------------


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make_client():
    sent = []
    clock = Clock()
    return PromptClient(sent.append, clock), sent, clock


def state(name, epoch):
    return json.dumps({"state": name, "epoch": epoch})


def test_a_new_client_is_quiet_and_the_latched_command_is_a_pause():
    client, sent, _ = make_client()
    client.before_spawn()
    assert json.loads(sent[-1]) == {"epoch": 1, "task": ""}
    assert client.is_quiet()          # nothing was ever asked of this process
    assert not client.ready


def test_idle_means_ready():
    client, _, _ = make_client()
    client.before_spawn()
    client.on_state(state("idle", 1))
    assert client.ready


def test_a_task_makes_it_not_quiet_until_a_pause_is_acknowledged():
    client, sent, _ = make_client()
    client.before_spawn()
    client.on_state(state("idle", 1))
    client.activate("pick up the lipstick")
    assert not client.is_quiet()
    epoch = client.pause()
    assert json.loads(sent[-1]) == {"epoch": epoch, "task": ""}
    assert not client.is_quiet()      # sent, not acknowledged
    client.on_state(state("idle", epoch - 1))
    assert not client.is_quiet()      # an older idle does not count
    client.on_state(state("idle", epoch))
    assert client.is_quiet()


def test_pausing_twice_sends_one_pause():
    client, sent, _ = make_client()
    client.activate("x")
    first = client.pause()
    assert client.pause() == first
    assert len(sent) == 2


def test_the_switch_time_is_reported_once_for_the_current_epoch():
    client, _, clock = make_client()
    epoch = client.activate("pick up the lipstick")
    clock.t += 0.2
    assert client.on_state(state("active", epoch - 1)) is None
    assert client.on_state(state("active", epoch)) == pytest.approx(0.2)
    assert client.on_state(state("active", epoch)) is None


def test_pause_waiting_counts_from_the_pause():
    client, _, clock = make_client()
    client.activate("x")
    assert client.pause_waiting_s() is None     # a task is running, not a pause
    client.pause()
    clock.t += 0.7
    assert client.pause_waiting_s() == pytest.approx(0.7)


def test_an_empty_task_is_not_a_task():
    client, _, _ = make_client()
    with pytest.raises(ValueError):
        client.activate("")


def test_garbage_state_is_ignored():
    client, _, _ = make_client()
    assert client.on_state("not json") is None
    assert not client.ready


# -- the mission rule ---------------------------------------------------------


def mission(switching="prompt", second=None, blend=None, params=None):
    base = {"task_description": "pick up the lipstick", "server_host": "10.0.0.1",
            "server_port": 5555, "checkpoint_label": "ck",
            "params": params or {"rtc_enable": True}}
    other = dict(base, task_description="scan and place the object")
    other.update(second or {})
    raw = {
        "name": "t",
        "policy_switching": switching,
        "policies": {"pick": base, "place": other},
        "steps": [
            {"step": "run_policy", "policy": "pick", "until": {"timeout_s": 5}},
            {"step": "run_policy", "policy": "place", "until": {"timeout_s": 5}},
        ],
    }
    if blend is not None:
        raw["prompt_blend"] = blend
    return Mission.from_dict(raw)


def test_process_is_the_default():
    raw = {"steps": [{"step": "wait", "seconds": 1}]}
    parsed = Mission.from_dict(raw)
    assert parsed.policy_switching == "process"
    assert parsed.prompt_blend is True


def test_prompts_of_one_checkpoint_may_share_a_client():
    parsed = mission()
    assert parsed.policy_switching == "prompt"
    assert [p.name for p in parsed.used_policies()] == ["pick", "place"]


@pytest.mark.parametrize("difference", [
    {"server_port": 5556},
    {"server_host": "10.0.0.2"},
    {"checkpoint_label": "other"},
    {"params": {"rtc_enable": False}},
])
def test_anything_but_the_prompt_differing_is_refused(difference):
    with pytest.raises(MissionError, match="ONE client"):
        mission(second=difference)


def test_the_orchestrator_owns_the_task_control_params():
    with pytest.raises(MissionError, match="set by the orchestrator"):
        mission(params={"rtc_enable": True, "task_control": False})


def test_prompt_switching_needs_prefetch():
    with pytest.raises(MissionError, match="prefetch_enable"):
        mission(params={"prefetch_enable": False})


def test_a_bad_mode_is_refused():
    with pytest.raises(MissionError, match="policy_switching"):
        mission(switching="restart")


def test_blend_must_be_a_bool():
    with pytest.raises(MissionError, match="prompt_blend"):
        mission(blend="yes")


def test_the_shipped_missions_still_parse():
    import glob

    for path in glob.glob("missions/*.yaml"):
        Mission.load(path)


# -- the runner passes the orchestrator's params last --------------------------


class _Log:
    def info(self, *_):
        pass

    warning = error = info


def test_extra_params_reach_the_client():
    runner = PolicyRunner(RunnerConfig(), _Log())
    policy = mission().policies["pick"]
    argv = runner.build_argv(policy, "run", {"task_control": True,
                                             "task_command_topic": "/c/task"})
    assert "task_control:=true" in argv
    assert "task_command_topic:='/c/task'" in argv


# -- the node's gate ----------------------------------------------------------


def _install_ros_stubs():
    """Enough of ROS to import orchestrator_node on a machine without it.

    Returns the names it added, so they can be taken out again once the node
    module is imported -- left in, a later ``importorskip("rclpy")`` would find
    the stub and run robot-only tests against it.
    """
    try:
        import rclpy  # noqa: F401

        return []
    except ImportError:
        pass
    added = []

    class _Anything(type):
        # ReliabilityPolicy.BEST_EFFORT and the like: any class attribute is
        # another placeholder.
        def __getattr__(cls, attr):
            return _Anything(attr, (), {})

        def __call__(cls, *a, **k):
            return types.SimpleNamespace()

    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        # Any other name is a harmless placeholder class.
        mod.__getattr__ = lambda attr: _Anything(attr, (), {})
        sys.modules[name] = mod
        added.append(name)
        return mod

    module("rclpy", init=lambda *a, **k: None, ok=lambda: True)
    module("rclpy.node", Node=type("Node", (), {}))
    for name in ("rclpy.qos", "rclpy.action", "geometry_msgs", "geometry_msgs.msg",
                 "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg",
                 "std_srvs", "std_srvs.srv", "control_msgs", "control_msgs.action",
                 "action_msgs", "action_msgs.msg", "nav_msgs", "nav_msgs.msg"):
        module(name)
    return added


_stubs = _install_ros_stubs()
from cerebel_orchestrator import orchestrator_node as node_module  # noqa: E402

for _name in _stubs:
    sys.modules.pop(_name, None)

Node = node_module.OrchestratorNode


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_):
        self.lines.append(msg)

    info = warning = error = _log


class _Policies:
    def __init__(self):
        self.running = False
        self.started = []
        self.stopped = []

    def check(self):
        return None

    def start(self, policy, label, extra_params=None):
        self.running = True
        self.started.append((policy.name, label, extra_params))

    def stop(self, reason):
        self.running = False
        self.stopped.append(reason)


class _Param:
    def __init__(self, value):
        self.value = value


def fake_node(blend=True):
    clock = Clock()
    sent = []
    self = types.SimpleNamespace()
    self.prompt_mode = True
    self.prompt_blend = blend
    self.prompt = PromptClient(sent.append, clock)
    self.policies = _Policies()
    self.mission = mission(blend=blend)
    self.runner = types.SimpleNamespace(faults=[])
    self.runner.fault = self.runner.faults.append
    self.logger = _Logger()
    self.get_logger = lambda: self.logger
    self.get_parameter = lambda name: _Param(f"/{name}")
    self._now = clock
    self.policy_startup_grace_s = 20.0
    self.prompt_pause_timeout_s = 1.0
    self._client_spawned_at = None
    self._run_stamp = "20260928_174335"
    self._log_label = types.MethodType(Node._log_label, self)
    self._spawn_prompt_client = types.MethodType(Node._spawn_prompt_client, self)
    return self, sent, clock


def action(kind, index=0):
    return types.SimpleNamespace(kind=kind, step=types.SimpleNamespace(index=index))


def gate(self, act):
    return Node._prompt_gate(self, act)


def test_a_policy_phase_waits_for_the_client_to_be_ready():
    self, sent, clock = fake_node()
    assert not gate(self, action("run_policy"))
    # The gate started the client, with task_control and the topics.
    name, label, extra = self.policies.started[0]
    assert name == "prompt_client"        # not the first policy's name
    assert label == "t_prompt_client_20260928_174335"   # stamped: runs do not overwrite
    assert extra["task_control"] is True
    assert json.loads(sent[0])["task"] == ""        # the latched command is a pause
    self.prompt.on_state(state("idle", 1))
    assert gate(self, action("run_policy"))


def test_a_client_that_never_gets_ready_faults_the_mission():
    self, _, clock = fake_node()
    gate(self, action("run_policy"))
    clock.t += 21.0
    assert not gate(self, action("run_policy"))
    assert self.runner.faults and "ready" in self.runner.faults[0]
    assert self.policies.stopped


def test_a_base_move_waits_for_the_pause_to_be_acknowledged():
    self, sent, clock = fake_node()
    gate(self, action("run_policy"))
    self.prompt.on_state(state("idle", 1))
    self.prompt.activate("pick up the lipstick")
    # A move after a running policy: the gate pauses and waits.
    assert not gate(self, action("move_base", 1))
    pause_epoch = json.loads(sent[-1])["epoch"]
    assert json.loads(sent[-1])["task"] == ""
    clock.t += 0.2
    assert not gate(self, action("move_base", 1))
    self.prompt.on_state(state("idle", pause_epoch))
    assert gate(self, action("move_base", 1))
    assert self.policies.stopped == []


def test_a_client_that_ignores_the_pause_is_killed_before_the_move():
    self, _, clock = fake_node()
    gate(self, action("run_policy"))
    self.prompt.on_state(state("idle", 1))
    self.prompt.activate("x")
    gate(self, action("move_base", 1))
    clock.t += 1.5
    assert gate(self, action("move_base", 1))
    assert self.policies.stopped == ["pause not acknowledged"]


def test_blending_keeps_the_client_running_into_the_next_policy():
    self, sent, _ = fake_node(blend=True)
    self.prompt.activate("pick up the lipstick")
    before = len(sent)
    Node._end_prompt_phase(self, action("run_policy", 0))   # next step is a policy
    assert len(sent) == before


def test_without_blending_a_phase_end_pauses():
    self, sent, _ = fake_node(blend=False)
    self.prompt.activate("pick up the lipstick")
    Node._end_prompt_phase(self, action("run_policy", 0))
    assert json.loads(sent[-1])["task"] == ""


def test_the_last_policy_phase_pauses_even_when_blending():
    self, sent, _ = fake_node(blend=True)
    self.prompt.activate("scan and place the object")
    Node._end_prompt_phase(self, action("run_policy", 1))   # nothing after it
    assert json.loads(sent[-1])["task"] == ""


def test_process_mode_gates_nothing():
    self, _, _ = fake_node()
    self.prompt_mode = False
    assert gate(self, action("move_base"))
    assert self.policies.started == []
