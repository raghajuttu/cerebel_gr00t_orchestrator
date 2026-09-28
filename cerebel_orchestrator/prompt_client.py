"""The orchestrator's half of a prompt-switched inference client.

``policy_switching: prompt`` runs ONE inference client for the whole mission
(adibot_gr00t_client with ``task_control:=true``) and switches its prompt at
each policy phase, instead of stopping one client and starting another. The
client starts idle, takes ``{"epoch", "task"}`` commands, and reports
``{"state", "epoch", ...}`` back -- see adibot_gr00t_client docs/TASK_CONTROL.md.

This class is the bookkeeping for that conversation, with no ROS in it:
``publish`` sends a command string, ``on_state`` takes a state string, and the
node asks it questions. It decides nothing about the mission.

Two facts the node relies on:

* **Quiet.** Before a step that is not a policy -- a base move above all -- the
  client must have stopped commanding the arms. ``is_quiet`` is true only when
  no task was ever sent to this process, or the client has ACKNOWLEDGED the
  latest pause. A pause that was sent but not acknowledged is not quiet: the
  node waits, and kills the client if the acknowledgement never comes.
* **Ready.** The client reports ``idle`` once its server ping succeeded. A
  phase does not start before that, so a cold start happens before the
  mission's first step rather than inside a phase's timeout.

Epochs only go up, across client restarts too. Before a client is (re)started
a pause is published, so the command the topic latches (TRANSIENT_LOCAL) is
never a task that a fresh client would pick up the moment it subscribes.
"""

from __future__ import annotations

import json
from typing import Callable, Optional


class PromptClient:
    def __init__(self, publish: Callable[[str], None], now: Callable[[], float]) -> None:
        self._publish = publish
        self._now = now
        self.epoch = 0
        self.ready = False
        self.state: Optional[str] = None
        self.reported_epoch = -1
        self.task: Optional[str] = None       # last task sent, None after a pause
        self._tasks_sent = 0                  # to THIS process
        self._pause_epoch: Optional[int] = None
        self._pause_sent_at: Optional[float] = None
        self._task_sent_at: Optional[float] = None

    # -- commands ------------------------------------------------------------

    def before_spawn(self) -> None:
        """A new client process is about to start: forget the old one, and
        make sure the latched command it will receive is a pause."""
        self.ready = False
        self.state = None
        self.reported_epoch = -1
        self._tasks_sent = 0
        self._send("")

    def activate(self, task: str) -> int:
        """Switch to (or start) ``task``. Returns the epoch it was sent under."""
        if not task:
            raise ValueError("an empty task is a pause; call pause()")
        self._tasks_sent += 1
        self._task_sent_at = self._now()
        return self._send(task)

    def pause(self) -> int:
        """Ask the client to stop commanding. No-op if it is already pausing."""
        if self.task is None and self._pause_epoch is not None:
            return self._pause_epoch
        return self._send("")

    def _send(self, task: str) -> int:
        self.epoch += 1
        self.task = task or None
        if not task:
            self._pause_epoch = self.epoch
            self._pause_sent_at = self._now()
        self._publish(json.dumps({"epoch": self.epoch, "task": task}))
        return self.epoch

    # -- state from the client -----------------------------------------------

    def on_state(self, text: str) -> Optional[float]:
        """Fold in one state report. Returns the seconds from the task command
        to the client's ``active`` report when this report completes the
        current switch, else None."""
        try:
            report = json.loads(text)
            state = str(report["state"])
            epoch = int(report["epoch"])
        except (ValueError, KeyError, TypeError):
            return None
        self.state = state
        self.reported_epoch = epoch
        if state == "idle":
            self.ready = True
        if (
            state == "active"
            and epoch == self.epoch
            and self.task is not None
            and self._task_sent_at is not None
        ):
            took = self._now() - self._task_sent_at
            self._task_sent_at = None
            return took
        return None

    # -- questions -----------------------------------------------------------

    def is_quiet(self) -> bool:
        """Is it certain the client is not commanding the arms?"""
        if self._tasks_sent == 0:
            return True
        return (
            self.task is None
            and self._pause_epoch is not None
            and self.state == "idle"
            and self.reported_epoch >= self._pause_epoch
        )

    def pause_waiting_s(self) -> Optional[float]:
        """Seconds since an unacknowledged pause was sent, or None."""
        if self.is_quiet() or self.task is not None or self._pause_sent_at is None:
            return None
        return self._now() - self._pause_sent_at
