"""Turning a sensor topic into a completion reading.

Every other condition in this package is proprioception: the grasp and settle
checks read ``/joint_states``, so they can only ever answer questions about the
robot. "Did the scanner read the barcode", "is there something in the box",
"did the weight arrive" are questions about the world, and no joint angle
answers them.

This is the layer that reads them. A mission declares what a signal is and
where it comes from; this subscribes, applies the signal's own rule to each
message, and hands the monitor a boolean.

The rule per type:

``bool``     ``std_msgs/Bool`` -- ``data`` as it stands.
``string``   ``std_msgs/String`` -- **any non-empty value is a firing.** That is
             what a barcode scanner is: it publishes nothing until it reads
             something, and then it publishes the code. The code itself is kept
             and printed in the phase's reason, so a run log says *which* item
             was scanned, not merely that something was.
``float``    ``std_msgs/Float64`` -- compared against ``above`` and ``below``.
             Both together mean a band, which is what a load cell wants: the
             weight arrived and is not absurd.

## What this deliberately does not do

It does not decide. It converts a message into a reading and gives it to
``PhaseMonitor``, where the arming, the latching and the conjunction with the
other conditions all live. A signal is not a shortcut past those rules -- it is
another input to them, and it can be ANDed with a gripper check exactly like
anything else.

It also does not verify that the sensor is alive. A scanner that is unplugged
publishes nothing, which is indistinguishable from a scanner that has not read
anything yet, and the phase runs to its timeout saying the signal never fired.
That is the honest outcome: the orchestrator cannot tell the difference, so it
does not pretend to.
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple

from .mission import Signal


def reading_from_bool(value: bool) -> Tuple[bool, str]:
    return bool(value), ""


def reading_from_string(value: str) -> Tuple[bool, str]:
    text = str(value).strip()
    return bool(text), text


def reading_from_float(value: float, signal: Signal) -> Tuple[bool, str]:
    reading = float(value)
    fired = True
    if signal.above is not None and reading < signal.above:
        fired = False
    if signal.below is not None and reading > signal.below:
        fired = False
    return fired, f"{reading:.3f}"


def interpret(signal: Signal, message) -> Optional[Tuple[bool, str]]:
    """One message -> ``(fired, detail)``, or None if it cannot be read.

    Pure, so every signal type is testable without a ROS graph.
    """
    data = getattr(message, "data", None)
    if data is None:
        return None
    if signal.type == "bool":
        return reading_from_bool(data)
    if signal.type == "string":
        return reading_from_string(data)
    if signal.type == "float":
        try:
            return reading_from_float(data, signal)
        except (TypeError, ValueError):
            return None
    return None


def message_type(signal: Signal):
    """The ROS message class for a signal's declared type.

    Imported lazily so the pure core -- and the mission validator -- stay
    importable without ROS on a laptop.
    """
    from std_msgs.msg import Bool, Float64, String

    return {"bool": Bool, "string": String, "float": Float64}[signal.type]


class SignalSource:
    """Subscribes to one signal's topic and forwards readings to a callback.

    The callback is set per phase: a phase with a ``signal`` condition points it
    at that phase's monitor, and everything else leaves it unset, so readings
    arriving between phases are dropped rather than latched into the next one.
    """

    def __init__(self, node, signal: Signal) -> None:
        self.signal = signal
        self.node = node
        self._sink: Optional[Callable[[bool, str], None]] = None
        self.last: Optional[Tuple[bool, str]] = None
        self.seen = 0
        node.create_subscription(
            message_type(signal), signal.topic, self._on_message, 10
        )

    def route_to(self, sink: Optional[Callable[[bool, str], None]]) -> None:
        self._sink = sink

    def _on_message(self, message) -> None:
        reading = interpret(self.signal, message)
        if reading is None:
            self.node.get_logger().warning(
                f"signal {self.signal.name!r}: unreadable message on "
                f"{self.signal.topic}",
                throttle_duration_sec=10.0,
            )
            return
        self.seen += 1
        self.last = reading
        if self._sink is not None:
            self._sink(*reading)
