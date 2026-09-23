# Nav2 with no SLAM

## What was removed, and what that leaves

The standard Nav2 bringup runs `map_server` (loads an occupancy grid), `amcl`
(corrects the pose against it), and a static costmap layer built from the map.
None of those are here. What is left:

| Component | Present | Why |
|---|---|---|
| `bt_navigator` | yes | provides the `navigate_to_pose` action the orchestrator calls |
| `planner_server` | yes | NavFn over an empty rolling costmap — effectively a straight line |
| `controller_server` | yes | the thing that actually produces velocities |
| `behavior_server` | yes | spin / back-up / wait recoveries |
| `local_costmap`, `global_costmap` | yes, **rolling windows** | rolling is what lets them exist without a map |
| `map_server` | **no** | there is no map |
| `amcl` | **no** | there is nothing to localise against |
| `slam_toolbox` | **no** | the robot does not need to learn the room |
| `obstacle_layer` | **no** | there is no scanner. This is the consequential one |

The global frame is `odom` in every parameter file. An identity `map -> odom`
static transform is published for RViz's benefit and nothing in the stack depends
on it.

## What this costs

**Station poses are dead reckoning.** `(0, 0, 0)` is wherever the wheels were when
the chassis driver came up, and every goal is measured from there through wheel
odometry alone. Error accumulates monotonically: it never gets corrected, because
nothing observes the world. Over a few metres of driving on a hard floor that is
often a centimetre or two; over a shift of repeated moves, with any wheel slip, it
is unbounded.

For this task that is survivable because the pick and place stations are one short
lateral move apart and the mission returns to `home` between cycles — but the
arms' tolerance is what decides. A GR00T policy fine-tuned from a fixed base sees
the table from a particular viewpoint; if the base stops 5 cm off, the policy's
first observation is out of the distribution it was trained on and the grasp
degrades in a way that looks like a policy problem rather than a navigation one.
That is why `xy_goal_tolerance` is 3 cm and not Nav2's default 25 cm.

**The robot is blind.** With no `obstacle_layer`, both costmaps are empty, so Nav2
will plan straight through a wall, a person, or a table leg and the controller will
drive into it at 0.15 m/s. The `max_linear` clamp in `base_adapter` and a human
watching are the only obstacle avoidance in this configuration. **Do not run an
unattended mission in a shared space.**

## What to add first, in order of value

1. **A 2D lidar and an `obstacle_layer`.** This is the single biggest upgrade.
   Add the layer to both costmaps in the parameter file and the robot stops being
   blind. The lidar also makes 2 and 3 possible.
2. **A saved map plus AMCL.** Map the area once with `slam_toolbox`
   (your [isaacsim-turtlebot3-slam-nav2](https://github.com/raghajuttu/isaacsim-turtlebot3-slam-nav2)
   repo already has that workflow), save it, then at run time use `map_server` +
   `amcl` with the map static. Still no SLAM at run time — but the pose is
   corrected, so station poses stop drifting and become properties of the room
   rather than of the power-on moment. Switch the global frame back to `map` and
   drop the identity transform.
3. **A fiducial marker at each station.** With `apriltag_ros` and a marker on each
   table, a step before the arms move can re-zero the base pose against the marker.
   This gives the tightest station repeatability of the three and is worth doing
   even *with* AMCL, because it corrects the pose in exactly the place where
   centimetres matter.

The orchestrator needs no changes for any of these. It sends poses in
`mission.frame_id` to `navigate_to_pose`; what maintains that frame is Nav2's
business.

## Choosing the parameter file

| Wheels | File | `base_adapter.allow_lateral` |
|---|---|---|
| Four driven wheels, no rollers — differential / skid-steer | `nav2_diff_drive.yaml` | `false` |
| Mecanum or omni wheels — can strafe | `nav2_holonomic.yaml` | `true` |

```bash
ros2 launch cerebel_orchestrator orchestrator.launch.py \
    mission:=nav_only kinematics:=holonomic
```

If you are not sure which you have, look at the tyres: rollers set at an angle
around the circumference mean mecanum. `probe_robot` guesses from controller and
topic names and says that it is guessing.

**On a differential base, "move left" is rotate, drive, rotate back.** That is
what `RotationShimController` is for: it turns in place until the base faces the
path, then hands over to `RegulatedPurePursuitController`. Expect the base to
pirouette before each move — that is the configuration working.

**On a holonomic base, "move left" is a lateral translation with the heading
held.** That is `vy_samples: 10` in the DWB section; without it DWB never
considers a sideways trajectory and turns to face every goal regardless of the
wheels. The alignment critics are deliberately weak for the same reason.

The kinematic assumption is enforced twice: once in the Nav2 parameters, and again
in `base_adapter`, which zeros `linear.y` unless `allow_lateral` is true. The
second one is what protects a differential base from being launched with the wrong
parameter file.

## Every velocity here is a first-run value

`max_linear: 0.15` m/s, `max_angular: 0.4` rad/s in the adapter, and matching
limits in the Nav2 files. They are slow on purpose: a blind robot with two arms
on top, being driven by a stack nobody has watched yet. Raise them only after
`nav_only` has run repeatedly with someone holding the hardware e-stop, and raise
the adapter's clamp and the Nav2 limits together — the clamp silently truncating
the planner's output makes the controller's behaviour hard to read.

## Recoveries

`spin`, `backup` and `wait` are enabled. With empty costmaps the usual trigger is
the progress checker: the base has not moved `required_movement_radius` (5 cm)
within `movement_time_allowance` (15 s), so Nav2 concludes it is stuck and tries
to recover. On a base whose odometry is wrong, or whose driver is ignoring the
twist, that is what you will see — a spin, a backup, then an aborted goal. It
usually means the adapter's output is not reaching the wheels; check
`base_adapter`'s forwarded/blocked counters before blaming the planner.
