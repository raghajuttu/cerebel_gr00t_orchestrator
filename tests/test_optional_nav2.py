"""Nav2 is optional, and the package must run without it.

The robot this ships to has no Nav2 installed -- it has no odometry for Nav2 to
use, so there was never a reason to. A mission made of run_policy and park_arms
steps never navigates, and must not be blocked by a package it does not need.

This is a regression test for a real failure: nav_client imported nav2_msgs at
module scope, orchestrator_node imported nav_client unconditionally, and the
whole node died at startup on a mission with no navigate step in it.
"""

import builtins
import importlib
import sys

import pytest

# nav_client needs rclpy, action_msgs and geometry_msgs whatever nav2 is doing,
# so these run on the robot and skip on a laptop. That is the right way round:
# the failure this guards against only ever happens where ROS is installed.
pytest.importorskip("rclpy", reason="ROS 2 not installed; these run on the robot")


@pytest.fixture
def without_nav2(monkeypatch):
    """Re-import nav_client with nav2_msgs unavailable."""
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name.startswith("nav2_msgs"):
            raise ImportError("No module named 'nav2_msgs'")
        return real_import(name, *args, **kwargs)

    for mod in [m for m in sys.modules if m.startswith("cerebel_orchestrator.nav_client")]:
        del sys.modules[mod]
    monkeypatch.setattr(builtins, "__import__", fake_import)
    try:
        yield importlib.import_module("cerebel_orchestrator.nav_client")
    finally:
        monkeypatch.setattr(builtins, "__import__", real_import)
        for mod in [m for m in sys.modules if m.startswith("cerebel_orchestrator.nav_client")]:
            del sys.modules[mod]


def test_nav_client_imports_without_nav2(without_nav2):
    assert without_nav2.NAV2_AVAILABLE is False
    assert without_nav2.NavigateToPose is None


def test_the_missing_message_names_the_alternative(without_nav2):
    """An operator reading this should learn what to do, not just what broke."""
    assert "nav2_msgs is not installed" in without_nav2.NAV2_MISSING
    assert "move_base" in without_nav2.NAV2_MISSING


def test_a_navigate_step_is_refused_with_that_message(without_nav2, monkeypatch):
    """Sending a goal is the one thing that genuinely cannot work."""

    class FakeNode:
        def get_clock(self):  # pragma: no cover - never reached
            raise AssertionError("send must refuse before touching the clock")

    client = without_nav2.NavClient.__new__(without_nav2.NavClient)
    client.node = FakeNode()
    client.action_name = "navigate_to_pose"
    client.available = False
    client._client = None
    with pytest.raises(RuntimeError, match="nav2_msgs is not installed"):
        client.send(station=None, frame_id="odom", timeout_s=1.0)


def test_server_ready_is_false_rather_than_a_crash(without_nav2):
    client = without_nav2.NavClient.__new__(without_nav2.NavClient)
    client._client = None
    assert client.server_ready(0.0) is False
