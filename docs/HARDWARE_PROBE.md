# The open hardware questions, and how to answer them

Three things about the base decide the configuration, and none of them can be read
off the code. Run the probe on the robot computer with the normal bring-up
running — it subscribes only, publishes nothing, calls nothing, and is safe next
to a live system:

```bash
ros2 run cerebel_orchestrator probe_robot --ros-args \
    -p duration_s:=10.0 -p out_file:=/tmp/base_adapter.probed.yaml
```

## Question 1 — does the chassis accept a velocity command?

The probe lists every `Twist` / `TwistStamped` topic with who publishes and who
subscribes to it. The line that matters is a topic with a **subscriber that is not
Nav2** — that subscriber is the chassis driver, and that topic is what
`base_cmd_topic` must be set to.

| What the probe shows | What it means | What to set |
|---|---|---|
| a twist topic with a vendor-node subscriber | the driver is Nav2-shaped already | `base_cmd_topic` to that topic, `base_cmd_type` to its type |
| twist topics but nobody subscribing | the bring-up is not running, or those topics are inputs to something dead | start the chassis bring-up and re-probe |
| no twist topics at all | no ROS velocity interface exists | a driver has to be written — see below |

If the chassis has no ROS interface, this package cannot drive it and no amount of
configuration will change that. What is needed then is a node that takes a
`Twist`, converts it to the vendor's wheel protocol (CAN, serial, a vendor SDK),
and publishes `nav_msgs/Odometry` back from the wheel encoders. That is a separate
piece of work, and it is the *only* part of this system that is blocked on it —
`policy_only` missions and the desk test both run without it.

## Question 2 — is there odometry, and is there a transform?

The probe prints every `Odometry` topic with the frames its messages actually
carry, and every TF edge it saw, then states plainly whether `odom -> base_link`
exists.

**Nav2 will not plan without that transform.** Odometry on a topic is not enough:
the costmaps and the controller look the robot up in TF. Two cases:

* the transform is present → `publish_odom_tf: false`, nothing to do.
* odometry is published but no transform → `publish_odom_tf: true`, and the
  adapter broadcasts it from the odometry message. Make sure nothing *else* is
  also broadcasting it; two publishers of one edge is a hard failure to diagnose.

If the frame names differ from `odom` / `base_link`, set `odom_frame` and
`base_frame` to match what the chassis actually uses and make the Nav2 parameter
file agree — `robot_base_frame` appears in five places in it.

## Question 3 — what are the wheels?

The probe guesses from controller and topic names (`mecanum`, `omni`, `diff`,
`skid`, `ackermann`) and says that it is guessing. **Confirm it mechanically:**

* rollers set at an angle around each tyre's circumference → mecanum, holonomic,
  can strafe → `nav2_holonomic.yaml`, `allow_lateral: true`
* plain tyres, four driven wheels → differential / skid-steer, cannot strafe →
  `nav2_diff_drive.yaml`, `allow_lateral: false`
* front wheels that steer → Ackermann; neither parameter file fits, and the
  planner needs a turning-radius constraint (`SmacPlannerHybrid`). Say so before
  going further.

If `ros2_control` is running, `ros2 control list_controllers` names the controller
plugin and settles it outright — `diff_drive_controller`, `mecanum_drive_controller`
and `ackermann_steering_controller` are unambiguous.

## Also checked, because they break a mission just as thoroughly

**All 16 canonical arm joints in `/joint_states`.** The probe compares against
`joints.py` and lists anything missing. A missing finger joint means the grasp
condition can never fire, and the phase will run to its timeout every time with
"never armed" in the reason. Extra joints are listed too — wheel joints showing up
here is a useful hint about the base.

**The gripper action servers and the Nav2 action server.** `navigate_to_pose`
missing means Nav2 is not up or is not activated (the lifecycle manager autostarts
it; if it is stuck, that is the thing to look at). The gripper actions missing
means `park_arms` cannot close the fingers — it will log a warning and leave them
as they are.

**The three camera topics.** Not used by the orchestrator, but the policy cannot
run without them, so their absence explains a client that exits at startup.

## The file it writes

```yaml
base_adapter:
  ros__parameters:
    base_cmd_topic: /qnbot/cmd_vel      # observed
    base_cmd_type: Twist                # observed
    vendor_odom_topic: /odom            # observed
    base_frame: base_link               # observed
    publish_odom_tf: true               # observed (no odom->base_link found)
    allow_lateral: false                # GUESS from names -- check the wheels
    max_linear: 0.15                    # deliberately slow, not observed
    max_angular: 0.4                    # deliberately slow, not observed
```

The header says which values came from observation and which are guesses. Copy the
observed ones into `params/orchestrator.yaml`; decide the rest yourself.
