# Every placeholder, and how to measure the real one

Nothing in `params/` has been measured on the robot. This is the list, worst
first.

## Gripper thresholds — `grasp_close_m`, `grasp_open_m`

**Placeholders: 0.010 and 0.030 metres.** These decide when a pick phase ends.

Measure them from a real run's per-tick CSV, written by `adibot_gr00t_client` into
`log_dir` and opened by
[`adibot_run_browser`](https://github.com/raghajuttu/adibot_run_browser). Plot the
finger joint (`openarm_left_finger_joint1`) across one successful pick and read
three numbers: open and empty, closed on the object, and released.

Put the thresholds **either side of the gap**, not at the extremes:

```
    0.000        0.008          0.018            0.038
      |------------|--------------|----------------|
   fully        on the         nothing           open
   closed       object        in between
               ^ grasp_close_m just above it      ^ grasp_open_m just below open
```

Too tight and the grasp never registers, so every pick runs to its timeout. Too
loose and the phase ends the moment the gripper starts closing on air. The failure
reason tells you which: `never armed` means the opposite state was never seen;
`finger at 0.0200 m` at the timeout means the thresholds bracket the wrong region.

These are gripper units in metres, the same units `GripperCommand.position` takes.
The recorder's `gripper_scale = 0.05` is already applied on the way in.

## Park poses — `params/park_poses.yaml`

**Placeholders: all zeros**, which on the OpenArm is the arm straight out. Zero is
there so the file parses, not so the robot uses it. See
[SAFETY.md](SAFETY.md#parking) for the procedure.

## Carry envelopes — `params/arm_envelopes.yaml`

**Placeholders: the full joint range**, which means the shipped envelopes pass on
any pose at all and are useless as a check until narrowed.

Measure them from real picks. Run the pick phase several times, let it end in its
carry pose with the object held, and record the canonical joint vector each time
(from `ros2 topic echo --once /joint_states`, reordered by name, or from the
inference client's per-tick CSV). For each joint that decides how far the arm
sticks out — shoulder pitch, elbow — take the range across runs and add a margin.
Leave the wrist and roll joints unconstrained unless they actually change the
envelope; the carry pose legitimately varies with where the object was, and an
envelope that is too tight turns a good pick into an aborted mission.

Set the finger bound first and tightest. It is what catches a dropped object
before the base drives to the next station with an empty hand.

## Robot radius — `robot_radius` in both costmaps

**Placeholder: 0.45 m.** Measure the circle that encloses the base *including the
arms in their carry pose, holding the object* — not the tucked pose, because the
carry pose is what the base drives in. Set both costmaps and the inflation radius
from it. With no obstacle layer this only affects the footprint Nav2 publishes —
until a lidar is fitted, at which point it becomes the number that decides
whether the robot fits through a gap.

## Velocities — `max_linear`, `max_angular`, and the Nav2 limits

**Placeholders: 0.15 m/s and 0.4 rad/s**, deliberately slow. Raise the adapter's
clamp and the Nav2 limits *together*: a clamp quietly truncating the planner's
output makes the controller's behaviour very hard to read.

## Goal tolerance — `xy_goal_tolerance`, `yaw_goal_tolerance`

**Set to 0.03 m / 0.05 rad**, against Nav2's default of 0.25 m. This is not a
navigation number, it is a manipulation number: it is how far from the trained
viewpoint the policy's first observation is allowed to be. Tighten it if grasps
degrade after a base move; loosen it only if the base genuinely cannot achieve it,
and expect to pay for it in grasp success.

## Motion threshold — `motion_eps`

**Placeholder: 0.05 rad/s.** It is a joint **speed**, not a per-tick delta: the
monitor divides by the interval between `/joint_states` stamps, so the number
means the same thing at any loop rate and can be carried between runs and robots.

Measure it as the apparent joint speed of a stationary arm. Take a log of the
robot holding still and, for each consecutive pair of samples, compute
`max|Δq| / Δt` across the 14 arm joints — that is the noise floor. Set
`motion_eps` above it and well below the slowest motion you would still call
"moving".

Only `settled` uses it. The two finger joints are excluded: they are in metres,
and a gripper still closing is not an arm still moving.

## Timeouts

`timeout_s` per policy phase (90 s in the shipped mission), `nav_timeout_s`
(120 s), `park_timeout_s` (60 s). Set each from a handful of successful runs:
long enough for a slow attempt, short enough that a failed one does not sit there.

`policy_startup_grace_s` (20 s) is how long the inference client is allowed to take
to connect to the policy server before a missing `/joint_states` counts against
the phase. It should exceed the worst startup you see in the client log — an
intercontinental SSH forward is not fast.

## Ramp speed — `max_joint_speed`

**Placeholder: 0.15 rad/s.** Start at 0.05 while checking poses. The ramp is a
straight line in joint space with no collision checking, so this is the number
that decides how much time you have to reach the e-stop.
