# Validation record

What each version was actually proven to do, and under which configuration. A
change that touches the control path — the orchestrator's step handling, the
adapter, the parker, the phase monitor — does not ship without an entry here.

The format follows `adibot_gr00t_client/docs/VALIDATION.md`: what ran, on what,
with which parameters, and what was observed.

## v0.1.0 — 2026-09-23

**Proven on hardware: nothing.** This is the first cut of the package.

Proven in unit tests (52, `python -m pytest tests -q`, no ROS required):

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
