# cerebel_gr00t_orchestrator

Task-level orchestration for an autonomous bimanual pick-and-place on a mobile
base: **Nav2 moves the base, GR00T N1.7 policies move the arms, and this package
decides which of them is allowed to run.**

**Version 0.3.0 — nothing here has run on hardware yet.** See
[Status](#status) for exactly what is and is not proven, and
[docs/BRINGUP.md](docs/BRINGUP.md) for the order to prove it in.

## What it does

A mission is a list of steps in a YAML file. The orchestrator executes them:

| Step | What runs | Base | Arms |
|---|---|---|---|
| `navigate` | Nav2 `NavigateToPose` to a named station | moving | holding a known pose |
| `run_policy` | one `adibot_gr00t_client` process, with that phase's parameters | gated shut | moving |
| `park_arms` | a joint-space ramp to a named pose | gated shut | moving |
| `check_arms` | verify the arms are inside a named joint envelope | gated shut | still |
| `wait` | nothing | gated shut | still |

```yaml
steps:
  - {step: park_arms, profile: home}                  # arms hanging down
  - {step: navigate, station: pick, on_fail: retry, retries: 2}
  - {step: park_arms, profile: ready}                 # start in distribution
  - step: run_policy                                  # pick, and hold it
    policy: pick_cube
    until: {timeout_s: 90.0, grasp: closed, side: left, settled: true, hold_s: 0.5}
    ends_parked: true
  - {step: check_arms, envelope: carry, seconds: 5.0} # verify the carry pose
  - {step: navigate, station: place}                  # drive, object in hand
  - step: run_policy                                  # place it
    policy: place_cube
    until: {timeout_s: 90.0, grasp: open, side: left, settled: true}
  - {step: park_arms, profile: home}                  # tuck back down
  - {step: check_arms, envelope: home, seconds: 5.0}
```

**The base carries the object in the policy's own carry pose.** There is no park
between the pick and the drive, because ramping the arms to a parked pose there
would drop what the gripper is holding. `ends_parked: true` declares that the
policy finishes somewhere drive-safe; `check_arms` measures whether it actually
did, against an envelope taken from real picks, and stops the mission before the
wheels turn if it did not — including when the object has been dropped, which
shows up as the finger joint being outside its bound.

Three design decisions shape everything else:

**The inference client is not modified.** `adibot_gr00t_client` was validated on
hardware as a node that runs one policy until it is killed. Rather than fork it,
the orchestrator runs it as a child process — one per policy phase, started with
that phase's `task_description`, server address and parameters. Switching policies
is starting a different process. Every phase gets its own run log, sidecar and
chunk store, named after the mission step. See
[docs/POLICY_SWITCHING.md](docs/POLICY_SWITCHING.md).

**The base is gated by a heartbeat, not by good intentions.** `base_adapter` sits
between Nav2 and the chassis and forwards wheel commands only while the
orchestrator is publishing `base_enable = true`, which it does only while a
`navigate` step is actually in flight. The gate is fail-closed on a 0.5 s timeout,
so the base stops if this node crashes, hangs, or is killed — the interlock does
not depend on the mission logic being right. See [docs/SAFETY.md](docs/SAFETY.md).

**Two loops, one thread.** An execution loop at 30 Hz sustains whatever is
running — the park ramp, the base-enable heartbeat — and decides nothing. A
supervision loop at 10 Hz asks whether the current step is done and switches to
the next, and commands nothing. Completion is fed from `/joint_states` at the
robot's own rate, so its thresholds are speeds rather than per-tick deltas and
stop meaning something different when a rate changes. Both loops are plain timers
on one single-threaded executor, which is why there are no locks anywhere in this
package. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#the-two-loops).

**A phase ends for a stated reason.** A GR00T policy never reports success; it
returns action chunks forever. So "the pick is done" is read off the robot — the
gripper closed on the object **and** the arm come to rest in its carry pose, or
an operator call — with a mandatory timeout behind it. Conditions combine with
AND, so a grasp registered mid-reach does not send the base off with the arm
still swinging. Every step reports `(ok, reason)`, and those reasons
are what the mission summary is made of. See
[docs/MISSIONS.md](docs/MISSIONS.md#when-a-policy-phase-is-done).

## Where it sits

```
        GPU box                          robot computer (SER9, ROS 2 Humble)
  ┌──────────────────┐            ┌──────────────────────────────────────────┐
  │ GR00T N1.7       │    ZMQ     │  orchestrator ──spawns──▶ adibot_gr00t_  │
  │ policy server    │◀──5555────▶│       │                   client         │
  │ (one checkpoint) │            │       │                      │           │
  └──────────────────┘            │       │ base_enable          ▼ arm cmds   │
                                  │       ▼                   arms + grippers│
                                  │  base_adapter ◀── /cmd_vel ── Nav2        │
                                  │       │                       ▲          │
                                  │       ▼ vendor twist          │ /odom, TF │
                                  │  4-wheel chassis ─────────────┘          │
                                  └──────────────────────────────────────────┘
```

This package contains the orchestrator, the adapter, the Nav2 configuration and
the mocks. It does **not** contain the policy, the inference client, the arm
bring-up or the chassis driver.

## Install

On the robot computer, next to the existing packages:

```bash
cd ~/Desktop/gripper/src && git clone <this repo> cerebel_gr00t_orchestrator
cd ~/Desktop/gripper && colcon build --packages-select cerebel_orchestrator && source install/setup.bash
```

The pure-Python core needs nothing but PyYAML, so the mission files and the
state machine can be checked anywhere:

```bash
python -m pytest tests -q
python -m cerebel_orchestrator.mission missions/*.yaml
```

## First things to run

**1. Find out what the robot actually exposes.** Read-only; publishes nothing.

```bash
ros2 run cerebel_orchestrator probe_robot --ros-args -p duration_s:=10.0
```

It prints the velocity-command topics and who listens to them, the odometry and
whether an `odom -> base_link` transform exists, whether there is a lidar,
whether all 16 canonical arm joints are present, which action servers are up, and
any hint of the wheel kinematics — then writes a `base_adapter` parameter file
from what it found. [docs/HARDWARE_PROBE.md](docs/HARDWARE_PROBE.md) explains how
to read it.

**2. Rehearse the whole mission against mocks.** No robot, no GPU box, real Nav2.

```bash
ros2 launch cerebel_orchestrator desk_test.launch.py mission:=two_station_pick_place
```

**3. Then the real thing, in this order** — base alone, policy alone, both:

```bash
ros2 launch cerebel_orchestrator orchestrator.launch.py mission:=nav_only
ros2 launch cerebel_orchestrator orchestrator.launch.py mission:=policy_only use_nav2:=false
ros2 launch cerebel_orchestrator orchestrator.launch.py mission:=two_station_pick_place
```

Nothing moves until you call `~/start`, unless `auto_start:=true`.
[docs/BRINGUP.md](docs/BRINGUP.md) is the full sequence with the checks at each
step.

## Controlling a run

```bash
ros2 topic echo /orchestrator/status                          # phase, step, reason
ros2 service call /orchestrator/start   std_srvs/srv/Trigger  # begin the mission
ros2 service call /orchestrator/hold    std_srvs/srv/Trigger  # stop everything
ros2 service call /orchestrator/resume  std_srvs/srv/Trigger  # restart the held step
ros2 service call /orchestrator/abort   std_srvs/srv/Trigger  # end the mission
ros2 service call /orchestrator/advance std_srvs/srv/Trigger  # finish this policy phase: success
ros2 service call /orchestrator/skip    std_srvs/srv/Trigger  # finish this policy phase: failure
ros2 topic pub /orchestrator/estop std_msgs/msg/Bool "{data: true}"   # software e-stop
```

The software e-stop is a convenience, not a safety device: it stops this node's
own commanding and shuts the base gate. It is not a substitute for the hardware
e-stop, which cuts power.

## The files

| Path | What it is |
|---|---|
| `cerebel_orchestrator/mission.py` | the mission file: parsing, strict validation, the park-before-navigate lint |
| `cerebel_orchestrator/state_machine.py` | control flow — order, retries, repeats, what a hold does. No ROS, no clock |
| `cerebel_orchestrator/phase_monitor.py` | when a policy phase is over: grasp, settle, timeout, and the arming rules |
| `cerebel_orchestrator/orchestrator_node.py` | the driver: the two loops, one mission, one thing moving at a time |
| `cerebel_orchestrator/policy_runner.py` | starting and stopping one inference client; the command line it builds |
| `cerebel_orchestrator/nav_client.py` | `NavigateToPose`, poll-shaped so nothing blocks |
| `cerebel_orchestrator/arm_park.py` | the joint-space ramp, and a standalone node to check a pose |
| `cerebel_orchestrator/base_adapter_node.py` | the interlock, the zero-holding, the clamps, the topic/type/TF shims |
| `cerebel_orchestrator/probe_robot.py` | the read-only hardware survey |
| `cerebel_orchestrator/envelopes.py` | named joint envelopes, and the check that the arms are inside one |
| `cerebel_orchestrator/joints.py` | the canonical 16-DOF order, mirrored from the inference client |
| `cerebel_orchestrator/fake_base.py`, `fake_arm.py` | the mocks the desk test runs against |
| `params/nav2_diff_drive.yaml`, `nav2_holonomic.yaml` | Nav2 without SLAM, one file per wheel type |
| `params/orchestrator.yaml`, `park_poses.yaml`, `arm_envelopes.yaml` | everything tunable, with every placeholder marked |
| `missions/` | `nav_only`, `policy_only`, `two_station_pick_place` |

## Documentation

| Document | What it covers |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | the three layers, who owns which actuator, why the client is a child process |
| [docs/BRINGUP.md](docs/BRINGUP.md) | the order to bring this up in, and the check that ends each step |
| [docs/HARDWARE_PROBE.md](docs/HARDWARE_PROBE.md) | the open hardware questions and how the probe answers them |
| [docs/NAVIGATION.md](docs/NAVIGATION.md) | Nav2 with no SLAM: what was removed, what it costs, what to add first |
| [docs/POLICY_SWITCHING.md](docs/POLICY_SWITCHING.md) | prompts, checkpoints, ports, and what a switch actually costs |
| [docs/MISSIONS.md](docs/MISSIONS.md) | the mission file, field by field, and every termination condition |
| [docs/SAFETY.md](docs/SAFETY.md) | the interlock, the e-stop, parking, and what each one does not protect against |
| [docs/TUNING.md](docs/TUNING.md) | every placeholder number, and how to measure the real one |
| [docs/VALIDATION.md](docs/VALIDATION.md) | what has actually been proven, and on what |

## Status

**v0.3.0 — written, tested in simulation of itself, never run on a robot.**

Proven: the pure-Python core, by 87 unit tests — mission validation, the step
sequencing, retries, repeats, hold-and-resume, the grasp and settle conditions
with their arming rules and their conjunction, the joint envelopes, the
independence of the completion check from both loop rates, and the
inference-client command line including the quoting of `task_description`.

Not proven, and not to be trusted until it is: every number in `params/` (all the
placeholders are marked), the Nav2 parameter sets against a real chassis, the base
adapter against a real vendor driver, the park poses (all zeros — deliberately
wrong), the carry envelope (the full joint range — deliberately useless until
measured), and the whole thing end to end. `enable_park` is off by default for this
reason: a `park_arms` step reports success without moving until you turn it on.

Open questions that the hardware decides, not the code — the chassis's ROS
interface, the wheel kinematics, and whether a lidar exists. All three are behind
parameters, and `probe_robot` answers the first and third. See
[docs/HARDWARE_PROBE.md](docs/HARDWARE_PROBE.md).

## License

Proprietary — © Cerebel. Built to sit alongside
[NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) (Apache-2.0) and
[Nav2](https://github.com/ros-navigation/navigation2) (Apache-2.0); neither is
vendored here.
