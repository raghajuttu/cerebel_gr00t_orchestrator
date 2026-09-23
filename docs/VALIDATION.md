# Validation record

What each version was actually proven to do, and under which configuration. A
change that touches the control path — the orchestrator's step handling, the
adapter, the parker, the phase monitor — does not ship without an entry here.

The format follows `adibot_gr00t_client/docs/VALIDATION.md`: what ran, on what,
with which parameters, and what was observed.

## v0.4.0 — 2026-09-23

**Proven on hardware: nothing.** Fitted to the one-checkpoint-on-cthor
deployment; see the changelog.

Proven in unit tests (96 total) — new in this release,
`tests/test_policy_preflight.py`:

| Area | What the tests cover |
|---|---|
| address parsing | `host:port`, bare host, bare port, whitespace, empty |
| one ping per server | two prompts against one checkpoint ping it once; two checkpoints are pinged separately |
| failure reporting | a dead server is named with its address and reason; one dead server among several is identified |
| the logger | one line per server, naming the policies that would have used it |
| robustness | a missing `pyzmq` is a reported failure, never an import crash |

**Not proven:** the ping against a real policy server. The protocol is copied
from the vendored client in `adibot_gr00t_client` (`{"endpoint": "ping"}`,
msgpack with `msgpack_numpy` hooks) and matches its framing byte for byte, but
nothing has exchanged a message with cthor yet. Step 5 of
[BRINGUP.md](BRINGUP.md) is where that gets confirmed, with
`ros2 run cerebel_orchestrator ping_policy`.

## v0.3.0 — 2026-09-23

**Proven on hardware: nothing.** Split into an execution loop and a supervision
loop; see the changelog.

Proven in unit tests (87 total, `python -m pytest tests -q`, no ROS required) —
new in this release, `tests/test_two_loops.py`:

| Area | What the tests cover |
|---|---|
| rate independence | `settled` fires at the same time (±one loop period) across every combination of 10/30/100 Hz sensor rates and 5/10/50 Hz supervisor rates |
| the bug this removes | a continuously moving arm is never called settled at any sampling rate — under the old per-tick-delta semantics a fast enough sample rate made each delta small enough to read as stationary |
| the split API | observing without ever deciding accumulates correctly; deciding without ever observing falls through to the timeout |
| clock defences | duplicate and out-of-order `/joint_states` stamps do not divide by zero or corrupt the speed |
| `stale_grace_s` | a gap in `/joint_states` inside the startup grace is survived; the same gap after it fails the phase |

**Not proven:** that 30 Hz is the right execution rate for the park ramp on real
controllers, and that 10 Hz supervision is soon enough to switch steps without a
visible pause between a finished pick and the base starting to move. Both are
one-line parameter changes and both should be looked at during step 8 of
[BRINGUP.md](BRINGUP.md).

## v0.2.0 — 2026-09-23

**Proven on hardware: nothing.** Reshaped around pick-and-carry; see the
changelog. Everything in the v0.1.0 entry below still applies, plus:

Proven in unit tests (71 total, `python -m pytest tests -q`, no ROS required):

| Area | What the tests cover |
|---|---|
| AND semantics | a grasp mid-reach does not end the phase; settling without the object does not either; both together do, and the reason names both |
| the settle check | a closing gripper is not an arm still moving |
| `ends_parked` | it satisfies the lint, and is refused without `settled` |
| `check_arms` | it satisfies the lint on its own; an envelope name is required |
| envelopes | bounds, unconstrained joints, every violation reported, a dropped object caught by the finger bound, and the shipped file parsing |
| the shipped mission | no park between the pick policy and the drive; `check_arms` in between |

**Still not proven, and specific to this release:** that a GR00T policy actually
comes to rest in a repeatable carry pose at all. The whole carry design rests on
it, and step 7b of [BRINGUP.md](BRINGUP.md) is where it gets measured. If the
carry pose turns out to vary too much to envelope, the fallback is a `park_arms`
to a *holding* profile between the pick and the drive — a pose that keeps the
gripper closed while moving the arm somewhere known.

## v0.1.0 — 2026-09-23

**Proven on hardware: nothing.** This is the first cut of the package.

Proven in unit tests (52 at the time, `python -m pytest tests -q`, no ROS required):

| Area | What the tests cover |
|---|---|
| mission validation | unknown keys, missing timeouts, grasp without a side, degrees in yaw, `params` shadowing the policy identity, retry/retries disagreement, cross-references to stations and policies |
| step sequencing | order, `repeat`, retries and their reset, `continue`, abort and fault being terminal |
| hold and resume | the current step restarts, no retry is consumed, the phase is restored |
| phase termination | grasp arming on the opposite state, hold windows, a bounce out of threshold, the untouched side, settle arming on motion and its grace window, timeout-as-success, stale `/joint_states`, operator advance and skip |
| the client command line | YAML quoting of `task_description`, type preservation of ints/bools/floats, the configurable command, refusing two concurrent sessions |
| the park lint | a policy between a park and a navigate, and the wrap-around for repeating missions |

**Not proven, and not to be trusted until it is:**

* every number in `params/` (see [TUNING.md](TUNING.md)) — all placeholders;
* the Nav2 parameter sets against a real chassis, in either kinematic variant;
* `base_adapter` against a real vendor driver — topic names, message type, and
  whether the `odom -> base_link` broadcast is needed;
* the park ramp on real arms. The poses are all zeros and `enable_park` is off;
* `PolicyRunner` against the real inference client: that the signal escalation
  shuts it down cleanly, and that a phase's run logs are indistinguishable from a
  hand-started run;
* the interlock's real timing — that the base actually stops within
  `enable_timeout_s` when the orchestrator is killed mid-move;
* anything end to end.

**Open questions the hardware decides:** the chassis's ROS interface, the wheel
kinematics, and whether a lidar exists. All three sit behind parameters;
`probe_robot` answers the first and third. See
[HARDWARE_PROBE.md](HARDWARE_PROBE.md).

### Template for the next entry

```
## vX.Y.Z -- YYYY-MM-DD

Ran: <mission> on <robot>, <N> times.
Configuration: kinematics=<...> enable_park=<...> policy=<checkpoint@port>
               grasp_close_m=<...> grasp_open_m=<...> max_linear=<...>
Observed: <what happened, with numbers -- success counts, station repeatability,
          phase durations, which condition ended each phase>
Logs: <run names in log_dir>
Still open: <what this run did not settle>
```
