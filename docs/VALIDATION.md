# Validation record

What each version was actually proven to do, and under which configuration. A
change that touches the control path — the orchestrator's step handling, the
adapter, the parker, the phase monitor — does not ship without an entry here.

The format follows `adibot_gr00t_client/docs/VALIDATION.md`: what ran, on what,
with which parameters, and what was observed.

## v0.5.0 — 2026-09-28

**The policy switch is proven on hardware.** First runs on Adibot against the
`pick_scan_place` checkpoint at 192.168.8.153:5555.

Ran: `policy_only`, then `pick_then_place`, several times each.
Configuration: `use_nav2:=false`, `enable_park:=false`, one checkpoint / three
prompts, `execution_horizon 26`, `prefetch_lead 16`, `rtc_overlap_steps 14`,
`rtc_frozen_steps 10`, `rtc_ramp_rate 5.0`, `rtc_enable true`,
`grasp_close_m 0.025`, `grasp_open_m 0.038`, `still_spread_rad 0.04`.

**Observed:**

| | |
|---|---|
| pick phase ended | `left gripper closed (0.0094 m) held 0.5s at 10.2s and arms inside envelope 'carry' for 0.5s at 10.2s` |
| place phase ended | `right gripper open (0.0481 m) held 1.0s at 20.3s and arms settled (moved 0.033 rad in 1.0s) at 20.3s` |
| `check_arms travel` | inside the envelope after 0.0s |
| client shutdown | **0.27 s** — so the switch delay is the new client's startup, not the old one's death |
| the arms across the switch | held the object with **nothing commanding them**. `hold_arms_between_policies` was false, so this confirms `forward_position_controller` latches, which ARCHITECTURE.md had always asserted and never tested |

Logs: `pick_then_place_c1_s*_run_policy_*` in `~/adibot_logs`.

**Five bugs the hardware found, all of them latent:**

1. `nav_client` imported `nav2_msgs` at module scope and the node imported it
   unconditionally, so the node could not start on a robot without Nav2 — even
   for a mission with no `navigate` step. Nav2 is optional now.
2. `_end_action` chose `info` or `error` into one call site. rclpy caches a
   logger's severity against the line it was called from, so the FIRST failing
   step of any mission raised instead of logging. Every step had succeeded until
   one did not.
3. The launch passed `park_poses_file` only when `enable_park` was true, so with
   parking off only the fallback profile existed and `park_arms home` failed
   mid-mission instead of being skipped.
4. The park poses were all zeros. Zero on this arm is not "hanging down"; a ramp
   to it would have swung both arms through a large unintended motion.
5. `still_spread_rad` was not declared as a node parameter, so the measured
   value in `params/` was silently ignored.

**Two behaviours corrected against the dataset** (302 episodes,
`20260923_pick_and_scan_and_place_01`):

*The pick could not end on the gripper.* Replayed against all 152 pick episodes,
`grasp: closed` fires in every one — but at 44% of a lipstick episode and 69% of
an oil or tissue one, while the arm is still down in the tote. `settled` fires at
70%, on pauses within the lift. A pose does it: gripper closed AND inside the
`carry` envelope fires in 151 of 152 at median 88%. Hence `envelope` as an
`until` condition.

*Stillness could not be a speed.* Teleop jitter wobbles a parked arm ~0.01 rad
between frames, which at 30 fps reads as 0.3 rad/s. The speed threshold ended
the place a median 1.9 s early — p95 7.5 s — in 75 of the 88 episodes that reach
a rest, which on hardware looked like the arm stopping mid-return. It is now a
position spread over the hold window: 0.024 rad from the pose the arm rests at,
against 0.143 before.

**Every threshold in `params/` is now measured, not guessed.** Gripper closed
0.006–0.019 m and open 0.045–0.050 m; the `carry`, `placed`, `ready` and `travel`
envelopes from the episodes those phases end and begin at; the `home`, `ready`
and `placed` park poses from the median frames. `docs/TUNING.md` still lists how
to re-measure each.

**Also fixed:** the missions carried `rtc_enable: false` with prefetch on, which
`adibot_gr00t_client/docs/VALIDATION.md` measures as the worst case for the seam
— 72.1 mrad against 47.7 for blocking, and `execution_horizon 16` re-planning
every 0.53 s rather than 0.87 s. Roughly two 4-degree snaps a second. Now the
validated RTC set. The operator reported the motion visibly smoother after this
change.

**Still not proven, and this is the whole of what movement rests on:**

* **Every `move_base` step.** Nothing has driven the base under the
  orchestrator. `move_only` exists to test it alone and to measure where the
  robot actually stops, which is the only way to know whether
  `move_scale_lateral: 1.05` — measured on a different floor in
  `move_base/move_base.py` — transfers.
* **`base_adapter` against the real chassis.** The interlock, the zero-holding,
  the fail-closed timeout, the crab trim: all unexercised. The chassis has no
  command watchdog, so the adapter's gate is the only thing that stops the
  wheels.
* **The park ramp on real arms.** It interpolates linearly in joint space, and
  `placed` to `home` moves right joint3 by 0.51 rad and joint5 by 0.68 rad —
  the right arm swinging back past the box. **There is no collision model in
  this package and that path has not been checked.** `two_item_kit` requires
  this ramp and aborts without it.
* **The `ready` envelope**, which is what verifies a park before a second pick.
  `carry` and `placed` have both fired on hardware; `ready` has not been seen.
* The place phase stops about 0.1 rad — six degrees — short of where place
  episodes end. Not the orchestrator cutting in: the demonstrations hold still
  for only 1.5–2 s, so requiring 2 s of stillness fires in 3 of 52 episodes and
  3 s in none. Either the `placed` park closes it, or
  `adibot_gr00t_client/scripts/pad_episode_tail.py` and a retrain do.
* The place client exits **code 1** where the pick exits 0, same binary, same
  signal. Unexplained.

**One question the dataset could not answer, since resolved by the operator:**
each pick was recorded with the base at its own spot — lipstick at 1, oil at 2,
tissue at 3 — and every place at spot 1. So the policy has seen each tote from
the base position it will be driven to, and driving back to spot 1 before a
place is required rather than incidental. `two_item_kit` and
`three_station_kit` are built on that.

Unit tests: 186, `python -m pytest tests -q`, no ROS required. New since v0.4.0:
the wheel-arc mover and its failure modes, the arm hold across a switch, the
sensor and envelope conditions, the optional-Nav2 import, and the park-profile
ordering.

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
