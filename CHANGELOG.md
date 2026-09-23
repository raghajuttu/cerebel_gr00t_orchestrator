# Changelog

Every version, and the issue that motivated it. One logical change per commit,
prefixed `feat:` / `fix:` / `docs:` / `chore:`.

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
