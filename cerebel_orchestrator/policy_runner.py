"""Starting and stopping one GR00T policy session -- i.e. switching policies.

The validated inference client (``adibot_gr00t_client``) is a node that runs one
policy, configured by ROS parameters at startup, until it is killed. Rather than
fork it to add a task interface, the orchestrator drives it as a **child
process**: one process per policy phase, started with that phase's parameters and
stopped when the phase ends. The client stays byte-identical to the version that
was validated on hardware, and every phase gets its own run log, sidecar and
chunk store for free.

What "switching policies" means concretely:

* **same checkpoint, different prompt** -- two policies pointing at one
  ``server_host:server_port`` with different ``task_description``. Cheap; the
  switch costs one client restart (a few seconds).
* **different checkpoint** -- two policies pointing at different ports. The
  policy server serves exactly one checkpoint, so run one server per checkpoint
  on the GPU box. Nothing here can make a running server load another
  checkpoint.

Stopping is where the care is. SIGINT is sent first so rclpy shuts the node down
the way Ctrl-C does; the arms then hold the last commanded position, because
``forward_position_controller`` latches its last command. Only if the client
ignores SIGINT does this escalate to SIGTERM and SIGKILL. An arm that is holding
a stale pose is the reason ``park_arms`` exists as its own mission step rather
than something the orchestrator does implicitly.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .mission import Policy

DEFAULT_POLICY_CMD: List[str] = [
    "ros2",
    "run",
    "adibot_gr00t_client",
    "inference_client",
]


def yaml_scalar(value: Any) -> str:
    """Render a Python value the way ``ros2 run -p name:=value`` wants it.

    The value half of ``-p name:=value`` is parsed as YAML, so a bare string
    that happens to look like a number would arrive as a number. Strings are
    therefore always quoted -- which also keeps ``task_description``, the one
    parameter that must match the training annotation verbatim, intact through
    the spaces it contains.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(yaml_scalar(item) for item in value) + "]"
    text = str(value)
    return "'" + text.replace("'", "''") + "'"


@dataclass
class Session:
    """A running (or mocked) policy session."""

    policy: Policy
    run_label: str
    argv: List[str]
    started_at: float
    process: Optional[subprocess.Popen] = None
    log_path: Optional[Path] = None
    _log_handle: Any = None
    stopping_since: Optional[float] = None

    @property
    def mocked(self) -> bool:
        return self.process is None

    def command_line(self) -> str:
        return " ".join(self.argv)


@dataclass
class RunnerConfig:
    policy_cmd: List[str] = field(default_factory=lambda: list(DEFAULT_POLICY_CMD))
    log_dir: str = "~/adibot_logs"
    mock: bool = False
    sigint_grace_s: float = 5.0
    sigterm_grace_s: float = 3.0
    startup_grace_s: float = 20.0


class PolicyRunner:
    """Owns at most one policy session at a time.

    ``logger`` is anything with ``.info`` / ``.warning`` / ``.error`` -- the ROS
    node's logger in production, ``logging`` in a test.
    """

    def __init__(self, config: RunnerConfig, logger) -> None:
        self.config = config
        self.log = logger
        self.session: Optional[Session] = None

    # -- starting ------------------------------------------------------------

    def build_argv(self, policy: Policy, run_label: str) -> List[str]:
        params: Dict[str, Any] = {
            "task_description": policy.task_description,
            "server_host": policy.server_host,
            "server_port": policy.server_port,
            "log_run_name": run_label,
            "log_dir": self.config.log_dir,
        }
        if policy.checkpoint_label:
            params["checkpoint_label"] = policy.checkpoint_label
        # The mission's own params come last so a phase can override the
        # defaults above -- but never the policy identity, which mission.py
        # already refuses to let params touch.
        params.update(policy.params)

        argv = list(self.config.policy_cmd) + ["--ros-args"]
        for name, value in params.items():
            argv += ["-p", f"{name}:={yaml_scalar(value)}"]
        return argv

    def start(self, policy: Policy, run_label: str) -> Session:
        if self.session is not None:
            raise RuntimeError(
                f"policy {self.session.policy.name!r} is still running; "
                "stop it before starting another"
            )
        argv = self.build_argv(policy, run_label)
        self.log.info(f"policy {policy.name}: {' '.join(argv)}")

        if self.config.mock:
            self.session = Session(
                policy=policy, run_label=run_label, argv=argv, started_at=time.monotonic()
            )
            self.log.warning(
                f"MOCK: not starting a real client for {policy.name}; "
                "the arms will not move"
            )
            return self.session

        log_dir = Path(os.path.expanduser(self.config.log_dir))
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{run_label}.client.log"
        handle = open(log_path, "wb")
        handle.write(("$ " + " ".join(argv) + "\n").encode())
        handle.flush()

        # start_new_session puts the client in its own process group, so a stop
        # signals the client and anything it spawned, and never this node.
        process = subprocess.Popen(
            argv,
            stdout=handle,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.session = Session(
            policy=policy,
            run_label=run_label,
            argv=argv,
            started_at=time.monotonic(),
            process=process,
            log_path=log_path,
            _log_handle=handle,
        )
        self.log.info(f"policy {policy.name} started as pid {process.pid}, log {log_path}")
        return self.session

    # -- watching ------------------------------------------------------------

    @property
    def running(self) -> bool:
        return self.session is not None

    def check(self) -> Optional[Tuple[bool, str]]:
        """None while the session is healthy; (False, reason) if it died.

        A client that exits by itself mid-phase has failed -- the usual causes
        are the policy server not answering the startup ping, a checkpoint whose
        modality config does not match the robot, or a missing camera topic. The
        last lines of its log are quoted into the reason so the mission summary
        is enough to tell what happened.
        """
        session = self.session
        if session is None or session.mocked:
            return None
        code = session.process.poll()
        if code is None:
            return None
        elapsed = time.monotonic() - session.started_at
        tail = self._log_tail(session)
        self._release(session)
        self.session = None
        return (
            False,
            f"inference client exited with code {code} after {elapsed:.1f}s{tail}",
        )

    def alive_for(self) -> float:
        return 0.0 if self.session is None else time.monotonic() - self.session.started_at

    # -- stopping ------------------------------------------------------------

    def stop(self, reason: str = "phase over") -> None:
        """Stop the session, escalating only as far as it has to.

        Blocks for at most ``sigint_grace_s + sigterm_grace_s``. It blocks on
        purpose: the next thing the mission does is move the base, and that must
        not begin while a policy is still publishing arm commands.
        """
        session = self.session
        if session is None:
            return
        self.session = None
        if session.mocked:
            self.log.info(f"MOCK: policy {session.policy.name} stopped ({reason})")
            return

        process = session.process
        self.log.info(f"stopping policy {session.policy.name} (pid {process.pid}): {reason}")
        for sig, grace in (
            (signal.SIGINT, self.config.sigint_grace_s),
            (signal.SIGTERM, self.config.sigterm_grace_s),
        ):
            if process.poll() is not None:
                break
            self._signal_group(process, sig)
            if self._wait(process, grace) is not None:
                break
        if process.poll() is None:
            self.log.error(
                f"inference client {process.pid} ignored SIGINT and SIGTERM -- sending SIGKILL. "
                "The arms hold their last commanded pose; park before moving the base."
            )
            self._signal_group(process, signal.SIGKILL)
            self._wait(process, 2.0)
        self.log.info(
            f"policy {session.policy.name} stopped with code {process.poll()} "
            f"after {time.monotonic() - session.started_at:.1f}s"
        )
        self._release(session)

    def _signal_group(self, process: subprocess.Popen, sig: int) -> None:
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except (ProcessLookupError, PermissionError):
            try:
                process.send_signal(sig)
            except ProcessLookupError:
                pass
        except AttributeError:
            # No process groups (Windows). Only reachable off-robot.
            process.send_signal(sig)

    @staticmethod
    def _wait(process: subprocess.Popen, timeout: float) -> Optional[int]:
        try:
            return process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    @staticmethod
    def _release(session: Session) -> None:
        if session._log_handle is not None:
            try:
                session._log_handle.close()
            except OSError:
                pass
            session._log_handle = None

    @staticmethod
    def _log_tail(session: Session, lines: int = 6) -> str:
        if session.log_path is None:
            return ""
        try:
            text = session.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        tail = [line for line in text.strip().splitlines() if line.strip()][-lines:]
        if not tail:
            return ""
        return " | last output: " + " / ".join(tail)


def describe_command(policy: Policy, run_label: str, config: RunnerConfig) -> str:
    """The command a phase would run -- for the dry-run printout and the docs."""
    return " ".join(PolicyRunner(config, _NullLogger()).build_argv(policy, run_label))


class _NullLogger:
    def info(self, *_args, **_kwargs) -> None:
        pass

    warning = error = info
