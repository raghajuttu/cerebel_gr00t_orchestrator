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

## Robot radius — `robot_radius` in both costmaps

**Placeholder: 0.45 m.** Measure the circle that encloses the base *including the
parked arms*, and set both costmaps and the inflation radius from it. With no
obstacle layer this only affects the footprint Nav2 publishes — until a lidar is
fitted, at which point it becomes the number that decides whether the robot fits
through a gap.

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

**Placeholder: 0.004 rad per tick.** Only used by `settled`. Measure it as the
per-tick joint noise of a stationary arm: take a log of the robot holding still
and look at the largest per-sample difference on any joint. Set it above that
noise floor and well below the smallest motion you would call "moving".

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
