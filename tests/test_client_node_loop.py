#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive the real InferenceClientNode control loop against a fake policy
server, with ROS and ZMQ stubbed out, and check what RTC actually sends and
logs. No robot, no ROS, no server:

    python -m unittest tests/test_node_loop.py -v     # or:  pytest tests/

The fake server reproduces the patched server's RTC contract on canonical
(H, 16) chunks: it copies the last `rtc_overlap_steps` rows it is sent onto the
new chunk's first steps and holds the first `rtc_frozen_steps` exactly. The
inference worker thread is not started; the test delivers each request's
response a fixed number of control ticks later, so timing is deterministic.
"""
import json
import os
import queue
import sys
import tempfile
import types
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# ---------------------------------------------------------------- ROS stubs
PARAM_OVERRIDES: dict = {}


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.infos, self.warns, self.errors, self.debugs = [], [], [], []

    def info(self, msg, **kw):
        self.infos.append(msg)

    def warn(self, msg, **kw):
        self.warns.append(msg)

    def error(self, msg, **kw):
        self.errors.append(msg)

    def debug(self, msg, **kw):
        self.debugs.append(msg)


class _Pub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg.data if isinstance(msg.data, str)
                              else list(msg.data))


class _Node:
    def __init__(self, name):
        self._params = {}
        self._logger = _Logger()

    def declare_parameter(self, name, default):
        self._params[name] = PARAM_OVERRIDES.get(name, default)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def get_logger(self):
        return self._logger

    def create_subscription(self, *a, **k):
        return None

    def create_publisher(self, *a, **k):
        return _Pub()

    def create_timer(self, *a, **k):
        return None

    def destroy_node(self):
        pass


class _ActionClient:
    def __init__(self, *a, **k):
        pass

    def server_is_ready(self):
        return False

    def send_goal_async(self, goal):
        return None


def _module(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


def _install_stubs():
    rclpy = _module("rclpy", init=lambda *a, **k: None, spin=lambda *a: None,
                    shutdown=lambda: None, ok=lambda: True)
    rclpy.node = _module("rclpy.node", Node=_Node)
    rclpy.action = _module("rclpy.action", ActionClient=_ActionClient)

    class QoSProfile:
        def __init__(self, **kw):
            pass

    rclpy.qos = _module("rclpy.qos", qos_profile_sensor_data=object(),
                        QoSProfile=QoSProfile,
                        ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1,
                                                                RELIABLE=2),
                        HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
                        DurabilityPolicy=types.SimpleNamespace(VOLATILE=1,
                                                               TRANSIENT_LOCAL=2))

    class GripperCommand:
        class Goal:
            def __init__(self):
                self.command = types.SimpleNamespace(position=0.0, max_effort=0.0)

    control = _module("control_msgs")
    control.action = _module("control_msgs.action", GripperCommand=GripperCommand)
    sensor = _module("sensor_msgs")
    sensor.msg = _module("sensor_msgs.msg", JointState=type("JointState", (), {}),
                         Image=type("Image", (), {}))

    class Float64MultiArray:
        def __init__(self):
            self.data = []

    std = _module("std_msgs")
    class String:
        def __init__(self, data=""):
            self.data = data

    std.msg = _module("std_msgs.msg", Float64MultiArray=Float64MultiArray,
                      String=String)
    _module("cv_bridge", CvBridge=type("CvBridge", (), {}))
    _module("cv2", resize=lambda img, size, interpolation=None: img, INTER_AREA=3)

    class _Sock:
        def setsockopt(self, *a):
            pass

        def connect(self, *a):
            pass

        def close(self, *a, **k):
            pass

    class _Ctx:
        @staticmethod
        def instance():
            return _Ctx()

        def socket(self, t):
            return _Sock()

    zmq = _module("zmq", Context=_Ctx, REQ=3, RCVTIMEO=1, SNDTIMEO=2, LINGER=4)
    zmq.error = _module("zmq.error", Again=type("Again", (Exception,), {}),
                        ZMQError=type("ZMQError", (Exception,), {}))
    _module("msgpack")
    _module("msgpack_numpy")


_modules_before = set(sys.modules)
_install_stubs()
try:
    from cerebel_orchestrator.gr00t_client.inference_client_node import (  # noqa: E402
        CANONICAL_JOINT_ORDER, InferenceClientNode)
finally:
    # The node has bound what it imported; take the stubs out of sys.modules so
    # a later importorskip("rclpy") in the orchestrator's robot-only tests is
    # not fooled into running against them -- also when the import failed, so
    # one broken import does not cascade into other test files.
    # (Orchestrator-only change to this test.)
    for _name in set(sys.modules) - _modules_before:
        if getattr(sys.modules[_name], "__file__", None) is None:
            del sys.modules[_name]

KEYS = (("left_arm", 0, 7), ("left_gripper", 7, 8),
        ("right_arm", 8, 15), ("right_gripper", 15, 16))
D = len(CANONICAL_JOINT_ORDER)


def to_dict(steps):
    """(H, 16) canonical -> the server's {key: (1, H, d)} form."""
    return {k: np.ascontiguousarray(steps[None, :, a:b], dtype=np.float32)
            for k, a, b in KEYS}


def from_dict(action):
    return np.concatenate([np.asarray(action[k])[0] for k, _, _ in KEYS], axis=-1)


class FakeServer:
    """A policy server whose plans move every joint at a constant velocity from
    the observed state, and which applies (or ignores) RTC the way the patched
    gr00t_policy.py does."""

    def __init__(self, H=40, honour_rtc=True, vel=0.01, bump=0.1):
        self.H, self.honour, self.vel, self.bump = H, honour_rtc, vel, bump
        self.requests = []
        self.prompts = []

    def get_action(self, obs, options):
        state = np.concatenate([np.asarray(obs["state"][k])[0, 0] for k, _, _ in KEYS])
        t = np.arange(1, self.H + 1, dtype=np.float32)[:, None]
        # A fresh plan starts a `bump` away from where the old one was heading
        # (alternating sign), the way a re-planned chunk disagrees with the
        # previous one at the seam. With RTC honoured the seed replaces it.
        bump = self.bump * (1 if len(self.requests) % 2 == 0 else -1)
        chunk = (state[None, :] + t * self.vel + bump).astype(np.float32)
        info = {}
        self.requests.append((obs.get("action"), options))
        self.prompts.append(
            obs["language"]["annotation.human.task_description"][0][0])
        if "action" in obs and self.honour:
            sent = from_dict(obs["action"])
            H0, ov = int(options["action_horizon"]), int(options["rtc_overlap_steps"])
            fr = int(options["rtc_frozen_steps"])
            assert sent.shape == (H0, D), sent.shape
            assert 0 < ov <= H0 and 0 <= fr <= ov, options
            seed = sent[H0 - ov:H0]
            chunk[:ov] = seed                     # frozen + ramp rows: copied
            tail = np.arange(1, self.H - ov + 1, dtype=np.float32)[:, None]
            chunk[ov:] = seed[-1][None, :] + tail * self.vel   # continue the plan
            info = {"rtc_applied": True, "rtc_options": dict(options)}
        return to_dict(chunk), info


class LoopSim:
    """Runs the node's control loop tick by tick with deterministic latency."""

    def __init__(self, params, server, latency_ticks, log_dir):
        PARAM_OVERRIDES.clear()
        PARAM_OVERRIDES.update({
            "enable_logging": True, "log_dir": log_dir, "log_run_name": "sim",
            "enable_limits": False, "enable_left_gripper": False,
            "enable_right_gripper": False, "checkpoint_label": "fake",
        })
        PARAM_OVERRIDES.update(params)
        self.server, self.latency = server, latency_ticks
        self.node = InferenceClientNode()
        n = self.node
        n.latest_state = np.zeros(D, dtype=np.float32)
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        n.latest_head = n.latest_wrist_left = n.latest_wrist_right = blank
        self.executed = []            # (seq, horizon_idx, step) per executed step
        self.pending = None           # (request, tick it is delivered on)
        orig = n._execute_step

        def spy(step, horizon_idx=-1):
            self.executed.append((n._current_inference_seq, horizon_idx, step.copy()))
            n.latest_state = step.astype(np.float32)   # perfect tracking
            orig(step, horizon_idx)
        n._execute_step = spy
        self.tick = 0

    def run(self, ticks):
        n = self.node
        for _ in range(ticks):
            if self.pending is not None and self.tick >= self.pending[1]:
                req, _ = self.pending
                self.pending = None
                action, info = self.server.get_action(req.obs, req.options)
                n._resp_q.put((req, action, info, 100.0))
            n._control_loop()
            try:
                req = n._req_q.get_nowait()
                self.pending = (req, self.tick + self.latency)
            except queue.Empty:
                pass
            self.tick += 1

    def finish(self):
        self.node.destroy_node()
        d = self.node.data_log
        with np.load(d.chunks_path) as z:
            store = {k: z[k] for k in z.files}
        with open(d.meta_path) as f:
            meta = json.load(f)
        return store, meta


class NodeLoopRtcTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def _check_alignment(self, sim, store):
        """Every seed starts right after the step the arm executed last before
        its request fired (prefetch: the request fires before that tick's
        step; blocking: the executed part is used up), taken from the chunk
        the seed claims to come from; the step executed on arrival is the copy
        of that plan step."""
        for i in range(len(store["seq"])):
            if not store["rtc_applied"][i] or store["req_tick"][i] == 0:
                continue
            seq_before, idx_before, _ = sim.executed[store["req_tick"][i] - 1]
            self.assertEqual(seq_before, store["rtc_prev_seq"][i])
            self.assertEqual(idx_before + 1, store["rtc_offset"][i])
            prev = np.flatnonzero(store["seq"] == store["rtc_prev_seq"][i])[0]
            ov, off, sk = store["rtc_overlap"][i], store["rtc_offset"][i], store["skip"][i]
            np.testing.assert_array_equal(store["rtc_seed"][i][:ov],
                                          store["chunks"][prev][off:off + ov])
            if sk < store["rtc_frozen"][i]:
                np.testing.assert_array_equal(store["chunks"][i][sk],
                                              store["chunks"][prev][off + sk])

    def test_prefetch_full_horizon_seeds_from_the_arms_step(self):
        sim = LoopSim({"prefetch_enable": True, "prefetch_lead": 12,
                       "execution_horizon": 40, "rtc_enable": True,
                       "rtc_overlap_steps": 12, "rtc_frozen_steps": 9},
                      FakeServer(H=40), latency_ticks=8, log_dir=self.dir)
        sim.run(200)
        store, meta = sim.finish()
        n = len(store["seq"])
        self.assertGreaterEqual(n, 6)
        self.assertEqual(list(store["rtc_applied"]), [0] + [1] * (n - 1))
        self.assertEqual(list(store["skip"][1:]), [8] * (n - 1))
        self.assertEqual(list(store["rtc_offset"][1:]), [28] * (n - 1))
        self.assertEqual(list(store["rtc_overlap"][1:]), [12] * (n - 1))
        self.assertEqual(list(store["rtc_frozen"][1:]), [9] * (n - 1))
        self.assertEqual(list(store["rtc_server_ack"]), [-1] + [1] * (n - 1))
        self.assertTrue(np.all(store["rtc_freeze_err_mrad"][1:] == 0.0))
        self.assertTrue(np.isnan(store["rtc_freeze_err_mrad"][0]))
        np.testing.assert_array_equal(store["rtc_prev_seq"][1:], store["seq"][:-1])
        self._check_alignment(sim, store)
        # The executed trajectory is one straight line: no jump at any seam.
        cmds = np.array([s for _, _, s in sim.executed])
        np.testing.assert_allclose(np.diff(cmds, axis=0), 0.01, atol=1e-5)
        self.assertEqual(meta["rtc_freeze_violations"], 0)
        self.assertEqual(meta["rtc_reduced_overlap"], 0)
        self.assertEqual(meta["rtc_frozen_overrun"], 0)
        self.assertEqual(meta["rtc_requests"], n - 1)
        self.assertEqual(meta["stale_chunks"], 0)
        self.assertEqual(sim.node.get_logger().errors, [])
        # obs_state is the state the request saw; the sidecar has the new param
        self.assertEqual(store["obs_state"].shape, (n, D))
        self.assertEqual(meta["rtc_freeze_warn_mrad"], 20.0)

    def test_short_execution_horizon_no_longer_seeds_from_the_tail(self):
        # v0.4.0 would have sent prev[28:40] here while the arm was on step 12.
        sim = LoopSim({"prefetch_enable": True, "prefetch_lead": 12,
                       "execution_horizon": 16, "rtc_enable": True,
                       "rtc_overlap_steps": 12, "rtc_frozen_steps": 9},
                      FakeServer(H=40), latency_ticks=8, log_dir=self.dir)
        sim.run(200)
        store, meta = sim.finish()
        n = len(store["seq"])
        self.assertEqual(store["rtc_offset"][1], 4)
        self.assertEqual(list(store["rtc_offset"][2:]), [12] * (n - 2))
        self.assertTrue(np.all(store["rtc_freeze_err_mrad"][1:] == 0.0))
        self._check_alignment(sim, store)
        cmds = np.array([s for _, _, s in sim.executed])
        np.testing.assert_allclose(np.diff(cmds, axis=0), 0.01, atol=1e-5)
        self.assertEqual(meta["rtc_freeze_violations"], 0)

    def test_server_that_ignores_the_seed_is_reported(self):
        sim = LoopSim({"prefetch_enable": True, "prefetch_lead": 12,
                       "execution_horizon": 40, "rtc_enable": True},
                      FakeServer(H=40, honour_rtc=False), latency_ticks=8,
                      log_dir=self.dir)
        sim.run(120)
        store, meta = sim.finish()
        n = len(store["seq"])
        self.assertTrue(np.all(store["rtc_freeze_err_mrad"][1:] > 20.0))
        self.assertEqual(list(store["rtc_server_ack"]), [-1] * n)
        self.assertEqual(meta["rtc_freeze_violations"], n - 1)
        errors = sim.node.get_logger().errors
        self.assertEqual(len(errors), 1)
        self.assertIn("RTC is NOT taking effect", errors[0])

    def test_blocking_mode_seeds_from_the_end_of_the_executed_part(self):
        sim = LoopSim({"prefetch_enable": False, "execution_horizon": 28,
                       "rtc_enable": True, "rtc_overlap_steps": 12,
                       "rtc_frozen_steps": 9},
                      FakeServer(H=40), latency_ticks=0, log_dir=self.dir)
        # blocking mode calls the server inline through the node's client
        sim.node.client = sim.server
        sim.run(150)
        store, meta = sim.finish()
        n = len(store["seq"])
        self.assertGreaterEqual(n, 5)
        self.assertEqual(list(store["skip"]), [0] * n)
        self.assertEqual(list(store["rtc_offset"][1:]), [28] * (n - 1))
        self.assertTrue(np.all(store["rtc_freeze_err_mrad"][1:] == 0.0))
        self._check_alignment(sim, store)
        cmds = np.array([s for _, _, s in sim.executed])
        np.testing.assert_allclose(np.diff(cmds, axis=0), 0.01, atol=1e-5)
        self.assertEqual(meta["rtc_freeze_violations"], 0)

    def test_overlap_is_cut_when_the_plan_is_nearly_used_up(self):
        # lead 6 < overlap 12 on a fully executed chunk: only 6 steps remain.
        sim = LoopSim({"prefetch_enable": True, "prefetch_lead": 6,
                       "execution_horizon": 40, "rtc_enable": True,
                       "rtc_overlap_steps": 12, "rtc_frozen_steps": 9},
                      FakeServer(H=40), latency_ticks=4, log_dir=self.dir)
        sim.run(160)
        store, meta = sim.finish()
        n = len(store["seq"])
        self.assertEqual(list(store["rtc_overlap"][1:]), [6] * (n - 1))
        self.assertEqual(list(store["rtc_frozen"][1:]), [6] * (n - 1))
        self.assertEqual(list(store["rtc_offset"][1:]), [34] * (n - 1))
        self.assertTrue(np.all(store["rtc_freeze_err_mrad"][1:] == 0.0))
        self._check_alignment(sim, store)
        self.assertEqual(meta["rtc_reduced_overlap"], n - 1)
        warns = [w for w in sim.node.get_logger().warns if "overlap cut" in w]
        self.assertEqual(len(warns), 1)

    def test_stale_chunk_is_dropped_and_never_becomes_the_seed(self):
        # A round trip longer than the whole chunk: the reply is stale.
        sim = LoopSim({"prefetch_enable": True, "prefetch_lead": 40,
                       "execution_horizon": 40, "rtc_enable": True},
                      FakeServer(H=40), latency_ticks=45, log_dir=self.dir)
        sim.run(91)
        node = sim.node
        # chunk 0 arrived at tick 45 (stall before it, skip 0) and became the
        # plan; the request fired right after it (cursor 0) came back at tick
        # 90 with all 40 steps executed -> dropped, plan untouched.
        self.assertEqual(node._stale_chunks, 1)
        self.assertEqual(node._last_seq, 0)
        self.assertEqual(node._chunk_cursor, 40)
        # Run until the next chunk (seq 2, requested without a seed at tick
        # 90) has been accepted at tick 135, but not until its successor --
        # which this deliberately hopeless configuration makes stale as well.
        sim.run(50)
        store, meta = sim.finish()
        self.assertEqual(meta["stale_chunks"], 1)
        self.assertEqual(list(store["seq"]), [0, 1, 2])
        self.assertEqual(list(store["rtc_applied"]), [0, 1, 0])
        self.assertEqual(store["skip"][1], 40)          # the stale one, kept on disk
        self.assertEqual(store["rtc_prev_seq"][1], 0)
        # the request after the drop had no plan left to stitch onto
        self.assertEqual(store["rtc_prev_seq"][2], -1)
        self.assertEqual(sim.node._last_seq, 2)



class TaskControlTest(unittest.TestCase):
    """task_control: one client, prompts switched at runtime (docs/TASK_CONTROL.md)."""

    PARAMS = {"task_control": True, "prefetch_enable": True, "prefetch_lead": 12,
              "execution_horizon": 40, "rtc_enable": True,
              "rtc_overlap_steps": 12, "rtc_frozen_steps": 9}

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = self._tmp.name
        self._sims = []

    def tearDown(self):
        # Close every run log first: Windows will not delete an open file.
        for sim in self._sims:
            if sim.node.data_log is not None:
                sim.node.data_log.close()
        self._tmp.cleanup()

    def _sim(self, latency=8, **params):
        sim = LoopSim(dict(self.PARAMS, **params), FakeServer(H=40),
                      latency_ticks=latency, log_dir=self.dir)
        self._sims.append(sim)
        # The state publisher is created only when task_control is on.
        self.assertIsNotNone(sim.node._task_state_pub)
        return sim

    @staticmethod
    def _command(sim, epoch, task):
        sim.node._on_task_command(
            types.SimpleNamespace(data=json.dumps({"epoch": epoch, "task": task})))

    @staticmethod
    def _states(sim):
        return [json.loads(m) for m in sim.node._task_state_pub.published]

    def test_it_starts_idle_and_commands_nothing(self):
        sim = self._sim()
        sim.run(60)
        self.assertEqual(sim.server.requests, [])
        self.assertEqual(sim.executed, [])

    def test_a_start_command_runs_that_prompt(self):
        sim = self._sim()
        sim.run(5)
        self._command(sim, 1, "pick up the lipstick")
        sim.run(40)
        self.assertEqual(set(sim.server.prompts), {"pick up the lipstick"})
        # Fired on the first tick after the command, so executing from the
        # 8-tick round trip onward: 40 - 8 steps.
        self.assertEqual(len(sim.executed), 40 - 8)
        states = self._states(sim)
        self.assertEqual([s["state"] for s in states], ["switching", "active"])
        self.assertEqual(states[-1]["epoch"], 1)

    def test_a_switch_keeps_the_arm_moving_and_stitches_the_seam(self):
        sim = self._sim()
        self._command(sim, 1, "pick up the lipstick")
        sim.run(30)                      # first chunk landed at tick 9
        executed_before = len(sim.executed)
        self._command(sim, 2, "scan and place the object")
        sim.run(30)
        # One step executed on EVERY tick across the switch: no pause.
        self.assertEqual(len(sim.executed), executed_before + 30)
        # The new prompt's request went out at once, seeded from the old plan.
        self.assertEqual(sim.server.prompts[-1], "scan and place the object")
        seed, options = sim.server.requests[-1]
        self.assertIsNotNone(seed)
        # With RTC honoured the trajectory is continuous through the switch.
        cmds = np.array([s for _, _, s in sim.executed])
        np.testing.assert_allclose(np.diff(cmds, axis=0), 0.01, atol=1e-5)
        store, meta = sim.finish()
        self.assertEqual([e["kind"] for e in meta["task_events"]], ["resume", "switch"])
        self.assertEqual(meta["superseded_chunks"], 0)

    def test_a_response_to_the_old_prompt_is_discarded(self):
        sim = self._sim()
        self._command(sim, 1, "pick up the lipstick")
        sim.run(30)
        # Let the next regular request fire (buffer down to prefetch_lead)...
        while not sim.node._inflight:
            sim.run(1)
        old_seq = sim.node._inference_seq - 1
        # ...and switch while it is in flight.
        self._command(sim, 2, "scan and place the object")
        sim.run(40)
        executed_seqs = {seq for seq, _, _ in sim.executed}
        self.assertNotIn(old_seq, executed_seqs)
        self.assertEqual(sim.server.prompts[-1], "scan and place the object")
        _, meta = sim.finish()
        self.assertEqual(meta["superseded_chunks"], 1)

    def test_a_pause_stops_commanding_at_once_and_drops_the_plan(self):
        sim = self._sim()
        self._command(sim, 1, "pick up the lipstick")
        sim.run(30)
        executed_before = len(sim.executed)
        self._command(sim, 2, "")
        sim.run(60)
        self.assertEqual(len(sim.executed), executed_before)
        self.assertEqual(sim.node._action_buffer, [])
        self.assertIsNone(sim.node._last_raw_action)
        self.assertEqual(self._states(sim)[-1]["state"], "idle")
        # Resuming starts clean: the first request carries no seed.
        self._command(sim, 3, "pick up the oil")
        sim.run(20)
        resumed = [i for i, p in enumerate(sim.server.prompts) if p == "pick up the oil"]
        self.assertTrue(resumed)
        self.assertIsNone(sim.server.requests[resumed[0]][0])
        self.assertEqual(self._states(sim)[-1]["state"], "active")

    def test_a_replayed_command_is_ignored(self):
        sim = self._sim()
        self._command(sim, 1, "pick up the lipstick")
        sim.run(30)
        self._command(sim, 2, "")
        sim.run(5)
        self._command(sim, 1, "pick up the lipstick")   # TRANSIENT_LOCAL replay
        sim.run(30)
        self.assertFalse(sim.node._active)
        self.assertEqual(sim.node._task_epoch, 2)

    def test_task_control_refuses_the_blocking_loop(self):
        sim = self._sim(prefetch_enable=False)
        self.assertFalse(sim.node.start())
        self.assertIn("prefetch_enable", sim.node.get_logger().errors[-1])

    def test_without_task_control_nothing_changes(self):
        sim = LoopSim({"prefetch_enable": True}, FakeServer(H=40),
                      latency_ticks=8, log_dir=self.dir)
        self._sims.append(sim)
        self.assertIsNone(sim.node._task_state_pub)
        sim.run(30)
        self.assertGreater(len(sim.executed), 0)


if __name__ == "__main__":
    unittest.main()
