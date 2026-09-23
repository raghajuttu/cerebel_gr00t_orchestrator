# Safety

Read this before turning on `enable_park` or raising any velocity.

## What actually protects you

**The hardware e-stop.** It cuts power. Nothing in this document replaces it, and
nobody should run a mission without it in reach.

Everything below is software, and software interlocks fail in the way software
fails: silently, and usually at the worst moment.

## The base gate

`base_adapter` forwards wheel commands only while the orchestrator is publishing
`base_enable = true` on `/orchestrator/base_enable`, and the orchestrator
publishes true only while a `navigate` step is actually in flight.

The gate is a **heartbeat, not a latch**. The adapter compares the last enable
message against `enable_timeout_s` (0.5 s), so:

* the orchestrator crashing, hanging, being killed, or being paused in a debugger
  shuts the gate within half a second;
* a stale `true` cannot hold the gate open;
* the interlock does not depend on the mission logic being correct — a bug that
  sends a Nav2 goal during a policy phase results in a base that does not move,
  not a base that moves while the arms are working.

While the gate is shut, the adapter publishes **zero twists continuously** rather
than going quiet, because most chassis drivers latch the last velocity they were
given. The same applies if Nav2 stops publishing mid-goal (`cmd_timeout_s`).

`require_enable: false` disables all of this. It exists for bringing up a chassis
driver on its own, before the orchestrator is in the picture. Do not leave it set.

## Clamping and the lateral block

Every forwarded twist is clipped to `max_linear` and `max_angular` at the last
point before the hardware, after Nav2, after any smoother. With
`allow_lateral: false` the lateral component is forced to zero and the drop is
logged.

This is the second enforcement of the kinematic assumption (the first is in the
Nav2 parameter file) and it is the one that protects a differential base from
being launched with `nav2_holonomic.yaml` by mistake.

## Driving with the object in hand

The shipped mission does not park between the pick and the drive: the pick policy
finishes holding the object in its carry pose, and that is the pose the base
travels in. Parking there would drop the object.

This is a real trade. A parked pose is one you chose and measured; a carry pose
is whatever a neural network happened to end in on this attempt. Three things
keep it honest, and all three have to be set up before the mission is run
unattended:

1. **The phase only ends when the arm has stopped** — `settled` alongside
   `grasp` in the `until` block. Without it the base could start moving while the
   arm was still swinging.
2. **`check_arms` verifies the pose** against a measured envelope before the
   navigate step, and fails the mission if the arm is somewhere else or the
   gripper has opened. See [MISSIONS.md](MISSIONS.md#carrying-the-object-ends_parked-and-check_arms).
3. **The costmap footprint has to enclose the carry pose**, not the tucked one.
   `robot_radius` is a placeholder; measure it with the arm holding something.

The first mission with a real object should be watched with a hand on the e-stop
for exactly this reason: the envelope is only as good as the runs it was measured
from, and the first few runs are the ones it was not measured from.

## Parking

`park_arms` is the only place the orchestrator commands the arms, and it is
**disabled by default**. With `enable_park: false` a park step logs what it would
have done and reports success without moving — so a mission can be rehearsed end
to end before anything is at stake.

What it does when enabled: a linear ramp in joint space from the measured pose to
the target, at `max_joint_speed` rad/s, published to the same
`forward_position_controller` topics the inference client uses, followed by one
`GripperCommand` goal per side.

What it does **not** do:

* **no collision checking** — not against the base, not against the other arm, not
  against the table. A park pose is only usable if every joint can move
  monotonically to it from anywhere the policy can leave the arm;
* **no planning** — the straight line in joint space is not a straight line in
  space;
* **no reachability check** — a typo of a radian is a fast move into the chassis.

Therefore, before setting `enable_park: true`:

1. Fill in `params/park_poses.yaml` from poses you have physically put the arms
   into and read off `/joint_states` (the message is scrambled — reorder by name).
2. Check each profile standalone, slowly, from several starting poses, with a hand
   on the e-stop:
   ```bash
   ros2 run cerebel_orchestrator arm_park --ros-args \
       -p enable_park:=true -p profile:=home \
       -p park_poses_file:=<path> -p max_joint_speed:=0.05
   ```
3. Only then enable it in a mission.

The shipped poses are all zeros. On the OpenArm that is the arm straight out —
deliberately a pose you would never travel with, so that an unedited file cannot
be mistaken for a configured one. The same goes for the envelopes in
`params/arm_envelopes.yaml`: they are set to the full joint range, so they pass on
anything until you narrow them.

## The hold path

Three things put the mission into `hold`: the `/orchestrator/estop` topic going
true, the `~/hold` service, and (indirectly) anything that makes the orchestrator
stop ticking.

A hold stops what is running: the policy process is killed, the Nav2 goal is
cancelled, the park ramp is frozen by re-commanding the measured pose, and the
base gate shuts. It does **not** relax the arms — they hold their pose, powered.

**Resuming restarts the current step from the beginning.** There is no sensible
way to resume a policy phase or a Nav2 goal from the middle after the actuators
have been cut, so the step is re-issued whole, and it does not consume a retry.
This puts a real constraint on mission design: a step has to be safe to run twice.
`navigate`, `park_arms` and `check_arms` are by construction; a `run_policy` step
is safe to repeat if the policy can recover from the state the interrupted attempt
left — which for a pick usually means the object is still somewhere the policy
can see it, and if it is already in the gripper, that the policy tolerates
starting while holding it.

Note the interaction with the grasp condition: a pick phase restarted with the
object already held can never arm `grasp: closed`, so it will run to its timeout.
Resuming a held pick phase generally means putting the object back first.

## The software e-stop is not an e-stop

`/orchestrator/estop` stops this node's commanding and shuts the base gate. It
does not cut power, does not stop a controller that is already tracking a
trajectory, and does not help if the robot computer has locked up. It is a
convenience for stopping a run from a terminal. The red button is the e-stop.

## Things this system does not protect against

* **Obstacles.** There is no scanner in the baseline configuration, so both
  costmaps are empty and Nav2 will plan through anything. See
  [NAVIGATION.md](NAVIGATION.md).
* **A policy doing something wrong.** The orchestrator watches the gripper and the
  clock. It does not know whether the arm is about to hit the table. The
  inference client's own `enable_limits` / `limits_file` is the joint-level guard,
  and it is off in the shipped missions because that is the validated
  configuration — turn it on with limits extracted from the dataset once the task
  is reliable.
* **Both arms at once.** The shipped mission disables the right arm
  (`enable_right_arm: false`) because the validated task is left-arm. A bimanual
  policy phase has two arms moving under one policy with no inter-arm collision
  checking anywhere in the stack.
* **An unattended run.** Do not.
