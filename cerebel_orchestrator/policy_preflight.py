"""Is the policy server actually reachable? Ask before the robot moves.

Without this, an unreachable server is discovered by the first ``run_policy``
step -- which in the shipped mission is step 3, after the base has already parked
the arms and driven to the pick station. The inference client then pings ten
times over twenty seconds, gives up, exits, and the phase fails. The robot is now
standing at the pick station having achieved nothing, and has to be walked back.

This matters more than it sounds for the cthor setup, where the "server" the
robot connects to is the local end of an SSH forward. ``127.0.0.1:5555`` accepts
a TCP connection whenever the tunnel process is alive, so the usual checks --
``ss -tlnp``, a successful ``connect()`` -- say yes even when the far end is gone.
Only a round trip proves the GPU is there. That is what this does: the same
``{"endpoint": "ping"}`` REQ that the inference client sends at startup, with the
same msgpack framing, from the same machine, through the same tunnel.

It speaks the protocol vendored in ``adibot_gr00t_client``; if that ever changes,
change it here too. ``pyzmq``/``msgpack`` are imported lazily so the rest of this
package -- and its tests -- keep working on a machine that has neither.

Standalone, for checking a tunnel before launching anything:

    python -m cerebel_orchestrator.policy_preflight 127.0.0.1:5555
    ros2 run cerebel_orchestrator ping_policy 127.0.0.1:5555
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

DEFAULT_TIMEOUT_MS = 5000


@dataclass(frozen=True)
class PingResult:
    host: str
    port: int
    ok: bool
    detail: str
    elapsed_s: float

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"

    def describe(self) -> str:
        mark = "OK  " if self.ok else "FAIL"
        return f"{mark} {self.address} ({self.elapsed_s * 1000:.0f} ms) {self.detail}"


def parse_address(text: str, default_port: int = 5555) -> Tuple[str, int]:
    """``host:port``, ``host`` or ``port`` -- whatever is easiest to type."""
    text = text.strip()
    if not text:
        raise ValueError("empty address")
    if ":" in text:
        host, _, port = text.rpartition(":")
        return (host or "127.0.0.1"), int(port)
    if text.isdigit():
        return "127.0.0.1", int(text)
    return text, default_port


def ping_server(host: str, port: int, timeout_ms: int = DEFAULT_TIMEOUT_MS) -> PingResult:
    """One round trip. Never raises -- the failure is the answer."""
    started = time.monotonic()

    try:
        import msgpack
        import msgpack_numpy
        import zmq
    except ImportError as exc:
        return PingResult(
            host,
            port,
            False,
            f"cannot check: {exc}. pyzmq/msgpack/msgpack_numpy are pip "
            "installs on the robot computer, the same ones the inference "
            "client needs.",
            0.0,
        )

    context = zmq.Context.instance()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.LINGER, 0)
    try:
        socket.connect(f"tcp://{host}:{port}")
        socket.send(
            msgpack.packb(
                {"endpoint": "ping"}, default=msgpack_numpy.encode, use_bin_type=True
            )
        )
        reply = msgpack.unpackb(
            socket.recv(), object_hook=msgpack_numpy.decode, raw=False
        )
        return PingResult(
            host, port, True, f"server replied {reply!r}", time.monotonic() - started
        )
    except zmq.error.Again:
        return PingResult(
            host,
            port,
            False,
            f"no reply within {timeout_ms} ms. The port accepts connections "
            "whenever the tunnel is up, so this usually means the far end -- "
            "the policy server on the GPU box -- is not running.",
            time.monotonic() - started,
        )
    except zmq.error.ZMQError as exc:
        return PingResult(
            host,
            port,
            False,
            f"{type(exc).__name__}: {exc}. Nothing is listening locally -- the "
            "SSH forward is probably down.",
            time.monotonic() - started,
        )
    finally:
        socket.close(linger=0)


def preflight(
    addresses: Dict[str, List[str]],
    timeout_ms: int = DEFAULT_TIMEOUT_MS,
    logger=None,
) -> List[PingResult]:
    """Ping every distinct server a mission uses.

    ``addresses`` is ``Mission.policy_ports()``: ``{"host:port": [policy, ...]}``.
    One ping per address, not per policy -- several prompts against one
    checkpoint is one server, and pinging it three times proves nothing extra.
    """
    results: List[PingResult] = []
    for address, policies in sorted(addresses.items()):
        host, port = parse_address(address)
        result = ping_server(host, port, timeout_ms)
        results.append(result)
        if logger is not None:
            names = ", ".join(sorted(policies))
            line = f"preflight {result.describe()} [{names}]"
            (logger.info if result.ok else logger.error)(line)
    return results


def failures(results: Sequence[PingResult]) -> List[PingResult]:
    return [result for result in results if not result.ok]


def summary(results: Sequence[PingResult]) -> str:
    bad = failures(results)
    if not bad:
        return f"all {len(results)} policy server(s) reachable"
    return "; ".join(f"{result.address}: {result.detail}" for result in bad)


def main(argv: Optional[List[str]] = None) -> int:
    args = [a for a in (sys.argv[1:] if argv is None else argv) if not a.startswith("--ros-args")]
    if not args:
        args = ["127.0.0.1:5555"]
    results = []
    for text in args:
        try:
            host, port = parse_address(text)
        except ValueError as exc:
            print(f"FAIL {text}: {exc}")
            results.append(PingResult(text, 0, False, str(exc), 0.0))
            continue
        result = ping_server(host, port)
        print(result.describe())
        results.append(result)
    return 1 if failures(results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
