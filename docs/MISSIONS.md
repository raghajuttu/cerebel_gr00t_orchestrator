# The mission file

A mission is data. Validate one without a robot:

```bash
python -m cerebel_orchestrator.mission missions/two_station_pick_place.yaml
# or, on the robot:
ros2 run cerebel_orchestrator mission_check <path>
```

Validation is strict: an unknown key is an error, not a shrug. A typo like
`statoin: pick` would otherwise become a navigate step with no station, discovered
on hardware.

## Top level

| Key | Meaning |
|---|---|
| `name` | used in log run names and the status topic |
| `frame_id` | the frame station poses are expressed in. `odom` in the no-SLAM configuration |
| `stations` | named base poses |
| `policies` | named GR00T sessions — see [POLICY_SWITCHING.md](POLICY_SWITCHING.md) |
| `steps` | the sequence, executed in order |
| `repeat` | how many times to run the whole list (default 1) |

## Stations

```yaml
stations:
  pick: {x: 0.0, y: 0.45, yaw: 0.0}
```

Metres and **radians**. A yaw outside ±2π is rejected with a message about
degrees, because that is the mistake it always is.

With odometry-only localisation these are measured from wherever the wheels were
when the chassis driver came up, so `home: {x: 0, y: 0, yaw: 0}` is by definition
the power-on pose.

## Steps

```yaml
steps:
  - {step: park_arms, profile: travel}
  - {step: navigate, station: pick, on_fail: retry, retries: 2}
  - step: run_policy
    policy: pick_cube
    until: {timeout_s: 90.0, grasp: closed, side: left, hold_s: 0.5}
  - {step: wait, seconds: 2.0}
```

| `step` | Fields | Notes |
|---|---|---|
| `navigate` | `station` | one `NavigateToPose` goal; fails on rejection, abort, or `nav_timeout_s` |
| `run_policy` | `policy`, `until` | one inference-client process for the life of the phase |
| `park_arms` | `profile` | a profile name from `params/park_poses.yaml` |
| `wait` | `seconds` | nothing moves |

Every step also takes `on_fail` and `retries`:

| `on_fail` | Behaviour |
|---|---|
| `abort` (default) | the mission ends, naming the step and the reason |
| `retry` | re-run the same step up to `retries` more times, then abort |
| `continue` | record the failure and move to the next step |

`retries` without `on_fail: retry` is an error, and vice versa — the combination
that silently does nothing is refused.

## When a policy phase is done

A GR00T policy never reports success; it returns action chunks forever. The
`until` block says what to watch instead.

```yaml
until: {timeout_s: 90.0, grasp: closed, side: left, hold_s: 0.5}
```

| Field | Meaning |
|---|---|
| `timeout_s` | **mandatory**. The only condition that always fires |
| `grasp` | `closed` or `open`, on `side`'s finger joint |
| `side` | `left` or `right` |
| `settled` | no joint moves more than `motion_eps` |
| `operator` | wait for the `~/advance` service |
| `hold_s` | how long the condition must hold before it counts (default 0.5 s) |

**If `until` has only a timeout, reaching it is success** — the step asked for "run
this policy for 90 seconds" and it did. With any other condition set, reaching the
timeout is a failure, and the reason says what the unmet condition was doing:

```
timed out after 90s: grasp left closed never armed, finger at 0.0200 m
```

### Arming

Both early conditions have to be *armed* before they can fire, because the state
they look for is usually true at the moment the phase starts.

* **`grasp` arms on seeing the opposite state.** A pick that must end closed has to
  have been open first, so the check cannot pass on the pose the previous phase
  left. If the gripper never opens, the phase runs to its timeout — which is the
  safe direction: the robot keeps working on the task rather than being declared
  done while standing still.
* **`settled` arms on seeing motion**, and never before `settle_grace_s` (3 s).
  Otherwise every phase would "settle" in the moment before its first action chunk
  arrives.

A phase also fails if `/joint_states` goes quiet for `stale_state_s` — the policy
would still be driving the arms off an observation nobody is checking.

The thresholds (`grasp_close_m`, `grasp_open_m`, `motion_eps`) are properties of
the robot, not of the mission, so they live in `params/orchestrator.yaml`. They
are placeholders until measured — see [TUNING.md](TUNING.md).

### `operator: true`

The phase ends when you call `~/advance` (success) or `~/skip` (failure). This is
what `missions/policy_only.yaml` uses, and it is the right setting for the first
few runs of a new task: watch one attempt, decide yourself, and use what you saw
to set a real condition.

## The park-before-navigate lint

After the inference client exits, the arms hold their last commanded pose. A
`navigate` step that is not preceded by `park_arms` therefore drives the base with
the arms wherever the policy stopped.

`mission_check` and the orchestrator's startup banner both warn about it. The
check walks backwards from each navigate to the last step that moved the arms —
`wait` and other `navigate` steps do not count — and it understands the wrap for a
repeating mission, where cycle 2 arrives at step 0 straight out of the last step of
cycle 1.

These are warnings, not errors: a short bench move with tucked arms is a
legitimate reason to skip parking.

## Shipped missions

| File | Purpose |
|---|---|
| `nav_only.yaml` | base only: left, home, right, home. No policy, no arm motion. The first thing to run on hardware |
| `policy_only.yaml` | one policy phase, operator-terminated, no base motion. The second |
| `two_station_pick_place.yaml` | the real task |
