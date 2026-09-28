"""Stop the inference client if the orchestrator dies.

The client is started in its own session (``start_new_session``) so that
stopping it can never signal the orchestrator. The cost of that isolation is
that nothing ties the client's life to the orchestrator's: a normal stop and
Ctrl-C are handled, but if the orchestrator is killed outright -- SIGKILL, the
OOM killer, a segfault in native code -- the client is orphaned and keeps
driving the arms with no phase condition that will ever end it.

The base has its heartbeat for exactly this case. This is the arms' version.

This process sits between the two, as the session leader of the client's
process group::

    orchestrator -> client_guard -> ros2 run ... inference_client
                    [ process group / session of the client ]

* It starts the client as its own child and exits with the client's exit code,
  so to the orchestrator it looks like the client itself.
* It **ignores SIGINT and SIGTERM**. A normal stop signals the whole group, the
  client shuts down, and the guard exits when it does -- never before, which is
  what keeps ``PolicyRunner.stop`` from starting the next client while the old
  one is still publishing. SIGKILL still takes everything down.
* If its parent dies it stops the group itself with the same escalation a
  normal stop uses -- SIGINT, then SIGTERM, then SIGKILL -- so the arms are left
  holding their last commanded pose, as after any stop.

Parent death is decided by one test only: ``getppid()`` no longer being the
orchestrator (the kernel reparents an orphan before anything else happens).
``PR_SET_PDEATHSIG`` is also set, but only to cut the poll short -- it fires
when the parent *thread* that forked us exits, which can happen while the
orchestrator is alive, so on its own it is not evidence of anything.

Only the standard library, and run by file path rather than ``-m``, so it starts
fast and does not depend on this package being importable from the child's
environment.

    python3 client_guard.py --parent PID --sigint-grace 5 --sigterm-grace 3 -- CMD...
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import List, Optional

POLL_S = 0.1


def _on_parent_death(_signum, _frame) -> None:
    # Installed so SIGUSR1 interrupts the sleep instead of killing the guard
    # (its default action). The getppid test in the loop decides.
    pass


def _set_pdeathsig(sig: int) -> None:
    """Ask the kernel to send ``sig`` when the parent exits. Linux only."""
    try:
        import ctypes

        libc = ctypes.CDLL(None, use_errno=True)
        PR_SET_PDEATHSIG = 1
        libc.prctl(PR_SET_PDEATHSIG, sig, 0, 0, 0)
    except (OSError, AttributeError):
        pass  # the getppid poll still covers it


def _signal_group(sig: int) -> None:
    try:
        os.killpg(os.getpgrp(), sig)
    except ProcessLookupError:
        pass


def _wait(child: subprocess.Popen, timeout: float) -> Optional[int]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        code = child.poll()
        if code is not None:
            return code
        time.sleep(POLL_S / 2)
    return child.poll()


def _stop_orphaned(child: subprocess.Popen, sigint_grace: float, sigterm_grace: float) -> int:
    sys.stderr.write(
        "client_guard: the orchestrator is gone -- stopping the inference client\n"
    )
    sys.stderr.flush()
    for sig, grace in ((signal.SIGINT, sigint_grace), (signal.SIGTERM, sigterm_grace)):
        _signal_group(sig)
        code = _wait(child, grace)
        if code is not None:
            return code
    _signal_group(signal.SIGKILL)  # takes this process with it
    return 137


def _exit_code(code: int) -> int:
    # Popen reports death-by-signal as -N; a shell would say 128+N.
    return 128 - code if code < 0 else code


def run(parent: int, sigint_grace: float, sigterm_grace: float, command: List[str]) -> int:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGUSR1, _on_parent_death)
    _set_pdeathsig(signal.SIGUSR1)

    # The child must get default handling back: an ignored SIGINT is inherited
    # across exec, and a client that ignored it could only ever be SIGKILLed.
    def _restore_defaults() -> None:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGUSR1, signal.SIG_DFL)

    try:
        child = subprocess.Popen(command, preexec_fn=_restore_defaults)
    except FileNotFoundError:
        sys.stderr.write(f"client_guard: command not found: {command[0]!r}\n")
        return 127

    while True:
        code = child.poll()
        if code is not None:
            return _exit_code(code)
        if os.getppid() != parent:
            return _exit_code(_stop_orphaned(child, sigint_grace, sigterm_grace))
        time.sleep(POLL_S)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--parent", type=int, required=True)
    parser.add_argument("--sigint-grace", type=float, default=5.0)
    parser.add_argument("--sigterm-grace", type=float, default=3.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("no command given after --")
    return run(args.parent, args.sigint_grace, args.sigterm_grace, command)


if __name__ == "__main__":
    sys.exit(main())
