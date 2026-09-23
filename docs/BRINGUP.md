# Bring-up, in order

Each step has a check that ends it. Do not start the next one until the check
passes — every one of these isolates a different thing, and doing them together is
how a bad station pose gets blamed on the policy.

## 0. Build and validate, anywhere

```bash
python -m pytest tests -q                        # 96 tests, no ROS needed
python -m cerebel_orchestrator.mission missions/*.yaml
```

On the robot computer:

```bash
cd ~/Desktop/gripper
colcon build --packages-select cerebel_orchestrator
source install/setup.bash
ros2 pkg executables cerebel_orchestrator        # 7 executables
```

**Check:** tests green, missions `OK`, executables listed.

## 1. Probe the robot

With the normal bring-up running (CAN up, arms up, cameras up, chassis up):

```bash
ros2 run cerebel_orchestrator probe_robot --ros-args -p duration_s:=10.0
```

**Check:** you can name the chassis's velocity topic and its type, you know
whether `odom -> base_link` exists, and all 16 canonical joints are present. Put
the observed values into `params/orchestrator.yaml`'s `base_adapter` block. Full
reading guide: [HARDWARE_PROBE.md](HARDWARE_PROBE.md).

## 2. Rehearse against mocks — no robot at all

Best done on a desk, or on the robot computer with the real bring-up **not**
running.

```bash
ros2 launch cerebel_orchestrator desk_test.launch.py mission:=two_station_pick_place
```

In another terminal:

```bash
ros2 topic echo /orchestrator/status
ros2 topic echo /orchestrator/base_enable
```

**Check:** the mission runs to `"phase": "done"`. The gate is true only during
navigate phases. The pick phase ends on the grasp condition (the fake gripper
closes at 8 s), not on its timeout. The summary lists every step with `ok`.

This validates the mission structure, the Nav2 parameters, the interlock and the
phase logic against a perfect robot. It proves nothing about the real one.

## 3. The base alone, on hardware

Arms untouched — `nav_only` has no policy phase, and `enable_park` is off, so
`park_arms` is a no-op.

```bash
ros2 launch cerebel_orchestrator orchestrator.launch.py \
    mission:=nav_only kinematics:=diff_drive
# then, with a hand on the e-stop:
ros2 service call /orchestrator/start std_srvs/srv/Trigger
```

Watch the adapter's counters (`gate: N forwarded, M blocked`) and
`/orchestrator/status`.

**Check:** the base reaches each station and Nav2 reports success; the gate is
open only while driving; the base stops within half a second when you kill the
orchestrator (`Ctrl-C` it mid-move — this is the interlock test, and it is worth
doing deliberately once).

If the base does not move: is the adapter forwarding (counters), is the driver
subscribed (`ros2 topic info <base_cmd_topic>`), is `odom -> base_link` there
(`ros2 run tf2_tools view_frames`)?

If Nav2 spins and then aborts: that is the progress checker. The commands are not
reaching the wheels, or the odometry is not moving.

## 4. Station poses

With the base where you want each station, read the pose:

```bash
ros2 topic echo --once /odom
```

Put `pose.pose.position.x`, `.y` and the yaw from the quaternion into the mission
file. Re-run `nav_only` until the base lands on each station repeatably. **This is
the step that decides whether the policy sees the scene it was trained on**, so
spend time here: 3 cm of goal tolerance is already a meaningful fraction of a
grasp's error budget.

## 5. The policy alone, no base motion

```bash
# on cthor: start the policy server on 5555
# on the robot: open the SSH forward, then prove it end to end
ros2 run cerebel_orchestrator ping_policy 127.0.0.1:5555

ros2 launch cerebel_orchestrator orchestrator.launch.py \
    mission:=policy_only use_nav2:=false
ros2 service call /orchestrator/start std_srvs/srv/Trigger
```

`ping_policy` is the same round trip the orchestrator's preflight makes before a
mission starts. A forward accepts connections whenever `autossh` is alive, so
this is the only check that tells "the tunnel is up" apart from "the GPU box is
answering".

The phase is `operator: true`, so it runs until you call `~/advance` or `~/skip`.

**Check:** the client starts, the banner in `~/adibot_logs/<run>.client.log` looks
exactly like a hand-started run, the arms move as they do when you run the client
yourself, and `~/advance` ends the phase and stops the client cleanly. Compare the
per-tick CSV against a hand-run log in `adibot_run_browser` — they should be
indistinguishable apart from the run name.

## 6. Measure the gripper thresholds

From the log of step 5, read the finger joint while open, while holding the
object, and after release. Set `grasp_close_m` and `grasp_open_m` either side of
the gap — see [TUNING.md](TUNING.md). Then re-run `policy_only` with
`until: {timeout_s: 90, grasp: closed, side: left}` and confirm the phase ends when
the object is picked up, not before and not on the clock.

## 7. Park poses

Follow [SAFETY.md](SAFETY.md#parking) exactly. Fill in `params/park_poses.yaml`
— `home` is the normal base pose with both arms hanging down, `ready` is where
the training episodes start — and check each profile standalone at
`max_joint_speed:=0.05`, from several starting poses, with a hand on the e-stop.

**Check:** each profile converges from anywhere a policy phase can leave the arm,
with nothing near the chassis on the way.

## 7b. The carry envelope

Run the pick phase a few more times and record where the arm ends up holding the
object. Narrow the `carry` envelope in `params/arm_envelopes.yaml` around those
runs, finger bound first — see [TUNING.md](TUNING.md#carry-envelopes).

**Check:** `check_arms [carry]` passes within a second on a good pick, and fails
with a named joint when you deliberately stop the phase early or take the object
out of the gripper. Test the failure case — an envelope that has never rejected
anything has not been shown to work.

## 8. Everything

```bash
ros2 launch cerebel_orchestrator orchestrator.launch.py \
    mission:=two_station_pick_place enable_park:=true
ros2 service call /orchestrator/start std_srvs/srv/Trigger
```

**Check:** the sequence runs; the base drives to the place station with the
object held in the pick policy's carry pose; `check_arms` passes before that
drive; each policy phase ends on its grasp-and-settled condition; the arms are
tucked back to `home` at the end. Then add `repeat: 2` to the mission and
run it twice in a row — that is where odometry drift shows up, and it is the first
real measure of whether dead reckoning is good enough for this task.

Record what each run proved in [VALIDATION.md](VALIDATION.md).
