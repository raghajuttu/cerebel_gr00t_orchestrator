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
  - {step: park_arms, profile: home}
  - {step: navigate, station: pick, on_fail: retry, retries: 2}
  - step: run_policy
    policy: pick_cube
    until: {timeout_s: 90.0, grasp: closed, side: left, settled: true, hold_s: 0.5}
    ends_parked: true
  - {step: check_arms, envelope: carry, seconds: 5.0}
  - {step: navigate, station: place}
```

| `step` | Fields | Notes |
|---|---|---|
| `navigate` | `station` | one `NavigateToPose` goal; fails on rejection, abort, or `nav_timeout_s` |
| `run_policy` | `policy`, `until`, `ends_parked` | one inference-client process for the life of the phase |
| `park_arms` | `profile` | a profile name from `params/park_poses.yaml` |
| `check_arms` | `envelope`, `seconds` | verify the arms are inside a named envelope. Nothing moves |
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
until: {timeout_s: 90.0, grasp: closed, side: left, settled: true, hold_s: 0.5}
```

**Conditions combine with AND.** The example above ends the phase when the
gripper is closed on the object *and* the arm has stopped moving -- which is the
question "has the policy finished the pick and come to rest holding it?", and it
is what has to be true before the base drives off with the object. A grasp
registered mid-reach, with the arm still swinging, does not end the phase.

| Field | Meaning |
|---|---|
| `timeout_s` | **mandatory**. The only condition that always fires |
| `grasp` | `closed` or `open`, on `side`'s finger joint |
| `side` | `left` or `right` |
| `settled` | no *arm* joint moves more than `motion_eps`. The fingers are excluded -- a gripper still closing is not an arm still moving, and they are not even in the same units |
| `operator` | no automatic end; the phase runs until `~/advance` or `~/skip`. Cannot be combined with `grasp` or `settled` |
| `hold_s` | how long each condition must hold before it counts (default 0.5 s) |

`~/advance` and `~/skip` work as an override in *any* policy phase;
`operator: true` declares that they are the only way this one ends.

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

## Carrying the object: `ends_parked` and `check_arms`

In the shipped mission the pick policy does not hand the object to anything — it
picks it up and comes to rest **holding it in its own carry pose**, and that pose
is what the base drives with. Parking in between would drop the object, so there
is deliberately no `park_arms` between the pick and the drive.

Two pieces of the mission file say so:

```yaml
- step: run_policy
  policy: pick_cube
  until: {timeout_s: 90.0, grasp: closed, side: left, settled: true, hold_s: 0.5}
  ends_parked: true          # a claim: this policy finishes drive-safe

- {step: check_arms, envelope: carry, seconds: 5.0}   # the measurement
```

`ends_parked: true` is a claim about the *checkpoint*, and it is what stops the
park-before-navigate lint complaining about the drive that follows. Because it is
only a claim, the schema refuses it unless the phase also waits for `settled` —
without that the phase could end mid-motion and the claim would mean nothing.

`check_arms` is the measurement that backs the claim up. It waits (up to
`seconds`) for every constrained joint to be inside a named envelope from
`params/arm_envelopes.yaml`, and fails the mission if they never are:

```
check_arms [carry] FAILED: envelope 'carry': openarm_left_joint2=-0.1800
outside [-1.6000, -0.8000]; openarm_left_finger_joint1=+0.0395 outside
[-0.0100, +0.0200]
```

That second violation is the gripper: the object was dropped. Catching it here
costs five seconds. Not catching it means driving to the place station and
running a place policy on an empty hand.

An envelope constrains only the joints it lists, which matters because the carry
pose legitimately varies with where the object was. Pin the joints that decide
how far the arm sticks out, plus the finger, and leave the rest free. See
[TUNING.md](TUNING.md#carry-envelopes).

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
`navigate` step therefore drives the base with the arms wherever the policy
stopped -- unless something has established where that is.

Three things do, and any of them satisfies the lint:

* `park_arms` -- the orchestrator put the arms somewhere known;
* `check_arms` -- the arms were verified to be inside an envelope;
* a `run_policy` marked `ends_parked: true` -- the policy finishes drive-safe.

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
