"""client_guard: the inference client must not outlive the orchestrator.

The process tests need process groups and signals, so they run on Linux (the
robot) and are skipped on a Windows desk machine.
"""

import os
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from cerebel_orchestrator import client_guard
from cerebel_orchestrator.policy_runner import PolicyRunner, RunnerConfig

GUARD = str(Path(client_guard.__file__).resolve())
posix_only = pytest.mark.skipif(os.name != "posix", reason="needs process groups")


class _Log:
    def info(self, *_):
        pass

    warning = error = info


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A zombie still answers kill(0); it is dead for our purposes.
    try:
        with open(f"/proc/{pid}/stat") as handle:
            return handle.read().split()[2] != "Z"
    except FileNotFoundError:
        return False


def _wait_for(predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


# -- the argv, everywhere ----------------------------------------------------


def test_the_guard_wraps_the_client_on_posix_only():
    runner = PolicyRunner(RunnerConfig(sigint_grace_s=4.0, sigterm_grace_s=2.0), _Log())
    argv = ["ros2", "run", "pkg", "exe", "--ros-args", "-p", "a:=1"]
    wrapped = runner.guarded(argv)
    if os.name != "posix":
        assert wrapped == argv
        return
    assert wrapped[1] == GUARD
    assert wrapped[wrapped.index("--parent") + 1] == str(os.getpid())
    assert wrapped[wrapped.index("--sigint-grace") + 1] == "4.0"
    # The client's own command is untouched, after the separator.
    assert wrapped[wrapped.index("--") + 1 :] == argv


def test_the_guard_can_be_turned_off():
    runner = PolicyRunner(RunnerConfig(guard_client=False), _Log())
    assert runner.guarded(["a", "b"]) == ["a", "b"]


def test_a_missing_command_is_refused():
    with pytest.raises(SystemExit):
        client_guard.main(["--parent", "1", "--"])


# -- the processes, on the robot ---------------------------------------------


@posix_only
def test_the_client_exit_code_passes_through(tmp_path):
    done = subprocess.run(
        [sys.executable, GUARD, "--parent", str(os.getpid()), "--",
         sys.executable, "-c", "raise SystemExit(3)"],
        start_new_session=True,
    )
    assert done.returncode == 3


@posix_only
def test_a_normal_stop_waits_for_the_client(tmp_path):
    """SIGINT to the group: the guard must outlive the client's shutdown, or the
    runner would start the next client while this one is still publishing."""
    client = textwrap.dedent(
        """
        import signal, sys, time
        def bye(*_):
            time.sleep(0.6)   # a slow, clean rclpy shutdown
            sys.exit(0)
        signal.signal(signal.SIGINT, bye)
        while True:
            time.sleep(0.05)
        """
    )
    guard = subprocess.Popen(
        [sys.executable, GUARD, "--parent", str(os.getpid()), "--",
         sys.executable, "-c", client],
        start_new_session=True,
    )
    time.sleep(0.5)
    os.killpg(os.getpgid(guard.pid), signal.SIGINT)
    time.sleep(0.3)
    assert guard.poll() is None, "the guard exited before its client had"
    assert guard.wait(timeout=5) == 0


@posix_only
def test_the_client_dies_when_the_orchestrator_is_killed(tmp_path):
    """The case this exists for: SIGKILL the parent, the client must stop."""
    pid_file = tmp_path / "client.pid"
    client = f"import os, time; open({str(pid_file)!r}, 'w').write(str(os.getpid())); time.sleep(60)"
    orchestrator = textwrap.dedent(
        f"""
        import os, subprocess, sys, time
        subprocess.Popen(
            [sys.executable, {GUARD!r}, "--parent", str(os.getpid()),
             "--sigint-grace", "1", "--sigterm-grace", "1", "--",
             sys.executable, "-c", {client!r}],
            start_new_session=True,
        )
        time.sleep(60)
        """
    )
    parent = subprocess.Popen([sys.executable, "-c", orchestrator])
    try:
        assert _wait_for(lambda: pid_file.exists() and pid_file.read_text(), 10)
        client_pid = int(pid_file.read_text())
        assert _alive(client_pid)

        parent.kill()  # SIGKILL: no cleanup code runs in the parent
        parent.wait()

        assert _wait_for(lambda: not _alive(client_pid), 5), (
            "the client outlived the orchestrator"
        )
    finally:
        if parent.poll() is None:
            parent.kill()
