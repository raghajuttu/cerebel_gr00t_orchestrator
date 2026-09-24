"""Keep the arms where the last policy left them, across a policy switch.

Switching policies means killing one inference client and starting another, and
the new one does not publish immediately -- it builds its limit arrays,
subscribes, and pings the policy server up to ten times before its 30 Hz loop
starts. For those seconds nobody is commanding the arms.

That gap is the whole problem with a pick-then-place cycle. The pick ends with
the arm up and the object in the gripper; the place policy has to start from
exactly there. Anything that resets, re-seeds or releases the arms in between
ends the run on the floor.

``forward_position_controller`` latches its last command, so in principle the
arms hold by themselves. This module does not rely on that. It republishes the
last commanded vector at the execution loop's rate, so a command is on the wire
every tick regardless of what the controller does when its input goes quiet.

**It republishes the last COMMANDED value, never the measured state.** The
follower on this robot sits a standing offset behind the command and never
catches up, so holding the measured position would command the arm to somewhere
it has already passed -- and would do it again at every switch, compounding.

## Why this is not a custom controller

The tempting fix for "the arm resets between policies" is to write a
replacement for ``forward_position_controller``. It is the wrong layer. The
reset comes from the controller being restarted, and the answer is to stop
restarting it: bring the arm up once, leave it up, and cycle only the client.
Then the controller is doing its job correctly and all that is missing is
somebody to talk to it during the gap. That is this file, and it is eighty
lines rather than a hardware interface.

## How it knows when to stop

It holds only while nothing else is commanding the arms, and the orchestrator
gates it on that directly.

The subtle case is a client that has started and then goes quiet. It does that
routinely: ``inference_client`` "holds pose" at a stall or a blocking chunk
boundary by publishing **nothing at all**, and in blocking mode that silence is
the whole round trip -- a quarter of a second. Those silences belong to the
client, not to this: the controller latches, the arm holds, and something else
stepping in would be competing with a policy that is merely thinking.

So within a policy phase the hold covers only the startup gap. The first message
the new client publishes stands it down for the rest of that phase, whatever
silences follow. ``begin_phase`` is what re-opens it for the next switch.

Distinguishing the client's messages from its own echo is done by value: a
message identical to what this just published is its own, and anything else is
somebody who should have the arms.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple


class ArmHold:
    """The last commanded arm pose, and whether it should be republished now.

    Pure: no ROS, no clock. The node feeds it messages and ticks, and it
    answers with the vectors to publish. ``now`` is supplied by the caller.
    """

    def __init__(self, foreign_quiet_s: float = 0.25) -> None:
        # How long the command topics must be quiet before a hold is allowed to
        # take over. Long enough that it never interleaves with a publisher that
        # is merely between ticks; short enough to cover a client's death.
        self.foreign_quiet_s = foreign_quiet_s
        self._commanded: dict = {"left": None, "right": None}
        self._published: dict = {"left": None, "right": None}
        self._foreign_at: Optional[float] = None
        self._holding = False
        # Has the current phase's client published yet? Reset at each policy
        # phase, set by the first message that is not this module's own echo.
        self._client_spoke = False

    # -- observation ---------------------------------------------------------

    def observe(self, side: str, values: Sequence[float], now: float) -> None:
        """Fold in one message seen on an arm command topic.

        A message equal to the last one this published is its own echo and
        changes nothing. Anything else is a real commander -- a client or the
        park ramp -- and both refreshes the pose to hold and stands the hold
        down.
        """
        vector = [float(v) for v in values]
        if not vector:
            return
        if self._published[side] is not None and vector == self._published[side]:
            return
        self._commanded[side] = vector
        self._foreign_at = now
        self._holding = False
        self._client_spoke = True

    def begin_phase(self) -> None:
        """A new policy phase is starting: the old client is gone and the new
        one has not spoken yet, so the hold is allowed again."""
        self._client_spoke = False

    @property
    def client_spoke(self) -> bool:
        """Has a commander published since the phase began?

        The orchestrator uses this to keep the hold out of a running policy's
        silences, which are the policy's own business.
        """
        return self._client_spoke

    @property
    def ready(self) -> bool:
        """Is there a pose to hold? False before any command has been seen."""
        return any(v is not None for v in self._commanded.values())

    @property
    def holding(self) -> bool:
        return self._holding

    def pose(self) -> dict:
        """The vectors that would be published, for logging and tests."""
        return {side: list(v) if v else None for side, v in self._commanded.items()}

    # -- the tick ------------------------------------------------------------

    def tick(self, now: float, allowed: bool) -> List[Tuple[str, List[float]]]:
        """``[(side, values)]`` to publish this tick. Empty means publish nothing.

        ``allowed`` is the orchestrator's answer to "is anything else entitled
        to the arms right now?" -- inverted. A hold never competes; it only
        fills a silence.
        """
        if not allowed or not self.ready:
            self._holding = False
            return []
        if self._foreign_at is None:
            return []
        if now - self._foreign_at < self.foreign_quiet_s:
            # Somebody was commanding very recently. Wait: a publisher between
            # two of its own ticks is not a silence.
            return []

        self._holding = True
        out: List[Tuple[str, List[float]]] = []
        for side in ("left", "right"):
            vector = self._commanded[side]
            if vector is None:
                continue
            self._published[side] = list(vector)
            out.append((side, list(vector)))
        return out

    def forget(self) -> None:
        """Drop the held pose. Used when the arms are known to have been moved
        by something this cannot see, so a stale pose is never re-commanded."""
        self._commanded = {"left": None, "right": None}
        self._published = {"left": None, "right": None}
        self._foreign_at = None
        self._holding = False
        self._client_spoke = False
