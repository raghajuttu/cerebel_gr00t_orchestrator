# Changelog

Every version, and the issue that motivated it. One logical change per commit,
prefixed `feat:` / `fix:` / `docs:` / `chore:`.

## v0.3.0 — 2026-09-23

Split the node into the **two loops** it should always have been: one that
sustains motion, one that decides when a step is done and switches. Doing that
exposed a real bug in the settle check.

**Changed**

* `orchestrator_node` now runs two timers instead of one.
  `_execution_tick` (`control_rate_hz`, 30 Hz) sustains what is running — the
  park ramp and the `base_enable` heartbeat — and decides nothing.
  `_supervisor_tick` (`supervisor_rate_hz`, 10 Hz) asks whether the step is done
  and switches, and commands nothing. Both are plain timers on the same
  single-threaded executor, so the split is in responsibilities, not in
  concurrency, and there are still no locks in this package.
  `tick_rate_hz` is gone; set the two rates instead.
* **`motion_eps` is now a joint speed in rad/s, not a per-tick delta** (default
  0.05). `PhaseMonitor` gained `observe` (fold in one `/joint_states` sample) and
  `verdict` (decide), and the orchestrator feeds `observe` from the subscription
  at the robot's rate rather than sampling the latest pose on its own tick.
  Previously the threshold really meant "how far a joint moved between two
  supervisor ticks", so the same number meant something three times stricter at
  30 Hz than at 10 Hz and could not be carried between runs. At a fast enough
  sampling rate it would have called a moving arm settled.
  `tests/test_two_loops.py` pins the independence across 10/30/100 Hz sensor
  rates and 5/10/50 Hz supervisor rates.
* `ArmParker` split the same way: `step` publishes the next ramp point, `outcome`
  decides. The ramp is now published at `control_rate_hz`, so its smoothness no
  longer depends on how often the mission logic is checked. `poll` remains as
  both, for the standalone `arm_park` node.
* `policy_startup_grace_s` now does something precise: it suppresses the
  `/joint_states` staleness check for that long after a policy phase starts
  (`MonitorConfig.stale_grace_s`), instead of being a special case in the poll
  path.

## v0.2.0 — 2026-09-23

Reshaped around **pick-and-carry**: the pick policy picks the object and comes to
rest holding it, the base drives in that carry pose, and the arms are tucked back
to the normal hands-down pose only at the end. The v0.1.0 mission parked between
the pick and the drive, which would have dropped the object.

**Added**

* `check_arms` step and `envelopes.py` — named per-joint envelopes with a
  measured `[min, max]` per joint, checked before the base is allowed to move.
  Unlisted joints are unconstrained, because a carry pose legitimately varies
  with where the object was. A finger bound in the envelope is what catches a
  dropped object before the robot drives to the next station with an empty hand.
* `ends_parked: true` on a `run_policy` step — a claim that the policy finishes
  in a drive-safe pose. It satisfies the park-before-navigate lint, and the
  schema refuses it unless the phase also waits for `settled`, so the claim
  cannot be made about a phase that could end mid-motion.
* `params/arm_envelopes.yaml`, wired through the launch file.

**Changed**

* **`until` conditions now combine with AND.** `{grasp: closed, settled: true}`
  ends the phase when the object is held *and* the arm has stopped — previously
  whichever fired first won, so a grasp registered mid-reach would have sent the
  base off with the arm still swinging.
* `settled` now ignores the two finger joints. They are in metres while the arm
  joints are in radians, so one `motion_eps` could not sensibly mean both, and a
  gripper still closing is not an arm still moving.
* `operator: true` can no longer be combined with `grasp` or `settled`; the two
  intentions do not mix. `~/advance` and `~/skip` still work as an override in
  any phase.
* `park_poses.yaml` profiles are now `home` (the normal base pose, arms hanging
  down), `ready` (where the training episodes start) and `travel` (for missions
  that do park before driving). `home` is what the mission tucks back to.
* `two_station_pick_place.yaml` rewritten to the carry sequence, with the pick
  phase set to `on_fail: abort`: a retry would re-run only that step, and a
  failed attempt that left the gripper closed can never re-arm `grasp: closed`.

## v0.1.0 — 2026-09-23

First cut. Nothing has run on hardware; see
[docs/VALIDATION.md](docs/VALIDATION.md).

**Added**

* `mission.py` — the mission file, with strict validation and the
  park-before-navigate lint. Runs anywhere; no ROS.
* `state_machine.py` — order, retries, repeats, hold-and-resume, terminal states.
  No ROS, no clock.
* `phase_monitor.py` — when a policy phase is done: grasp, settle, timeout,
  operator, and the arming rules that stop a condition firing on the pose the
  phase started in.
* `orchestrator_node.py` — the driver. One timer, poll-shaped handlers, six
  Trigger services, a JSON status topic, and the `base_enable` heartbeat.
* `policy_runner.py` — a policy phase as a child `adibot_gr00t_client` process:
  the command line it builds, the signal escalation that stops it, and the
  client's own log quoted into a failure reason.
* `nav_client.py` — `NavigateToPose`, non-blocking, with its own timeout.
* `arm_park.py` — the joint-space ramp, and a standalone node to check a pose.
  Disabled by default.
* `base_adapter_node.py` — the fail-closed interlock, zero-holding, clamping, the
  lateral block, and the topic/type/TF shims.
* `probe_robot.py` — a read-only survey that answers the open hardware questions
  and writes an adapter parameter file from what it found.
* `fake_base.py`, `fake_arm.py` and `desk_test.launch.py` — the whole stack,
  including real Nav2, against mocks.
* Nav2 parameter sets for differential and holonomic bases, with no SLAM, no map
  server and no AMCL.
* Three missions: `nav_only`, `policy_only`, `two_station_pick_place`.
* Nine documents, including a bring-up order and a list of every placeholder
  number with how to measure the real one.
