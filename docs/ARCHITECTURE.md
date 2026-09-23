# Architecture

## Three layers

| Layer | Runs where | Owns |
|---|---|---|
| **Task** | this package, robot computer | which phase is current, which policy is loaded, whether the base may move |
| **Skill** | `adibot_gr00t_client` (child process) + Nav2 | arm trajectories from a policy; base trajectories from a planner |
| **Hardware** | `openarm_bringup`, the vendor chassis stack, `ros2_control` | motors, encoders, CAN |

The task layer never commands a motor, with one exception: `park_arms` publishes
arm positions directly, because there is no policy for "tuck the arms" and a
planner for two arms would be a project of its own. That exception is why parking
is off by default and has its own document section
([SAFETY.md](SAFETY.md#parking)).

## Who owns which actuator, and when

At any moment exactly one thing is allowed to move:

```
phase        base            arms                         who commands the arms
----------   -------------   --------------------------   ---------------------
navigate     Nav2            holding their parked pose    nobody
run_policy   gate shut       policy                       the child process
park_arms    gate shut       ramp                         the orchestrator
wait         gate shut       holding                      nobody
hold/estop   gate shut       holding                      nobody
```

"Holding" is not a null state. `forward_position_controller` latches its last
command, so after the inference client exits the arms stay exactly where the last
action chunk put them, powered and stiff. That is the whole reason `park_arms`
exists as an explicit step: a `navigate` that is not preceded by one drives the
base with the arms wherever the policy happened to stop. `mission.py` lints for
it and the orchestrator prints the warnings at startup.

## Why the inference client is a child process

`adibot_gr00t_client` is a node that takes its policy from ROS parameters at
startup and runs until it is killed. To switch policies, something has to change
those parameters, and there are three ways to do it:

| Approach | Cost |
|---|---|
| **Child process per phase** (chosen) | a few seconds of restart per switch; the client stays byte-identical to the validated version; each phase gets its own logs for free |
| Add a task action server to the client | no restart cost, but forks a file that took two rounds of hardware debugging to get right, and every future upstream change has to be re-merged |
| Set parameters at runtime on a long-lived client | the client reads its parameters once, in `__init__`; making them dynamic is the same fork as above, plus a mid-run reconfiguration path that nothing validates |

The restart cost is real but it lands in the right place. A policy switch in this
task happens between a pick and a place, with a base move in between — there is
already a pause there. What the choice buys is that the thing driving the arms on
hardware is the thing that was tested on hardware.

Consequences worth knowing:

* **Stopping blocks.** `PolicyRunner.stop` waits for the process to be gone
  before returning (SIGINT, then SIGTERM, then SIGKILL, with grace periods). This
  is deliberate: the base must not start moving while another process is still
  publishing arm commands.
* **A dead client is a failed phase.** If the client exits on its own — no policy
  server, a modality-config mismatch, a missing camera topic — the phase fails and
  the last lines of the client's log are quoted into the reason.
* **The command line is the interface.** It is built in
  `PolicyRunner.build_argv` and tested in `tests/test_policy_runner.py`, because a
  `task_description` that loses its spaces is a phase that silently runs the wrong
  policy.

## Why nothing blocks in the node

The whole mission advances inside one timer callback at 10 Hz on a single-threaded
executor. Every handler is poll-shaped: `send` then `poll`, never `wait`. That is
what keeps the e-stop subscription and the six service calls live while a 90-second
policy phase is running. `PolicyRunner.stop` is the one place that blocks, and it
blocks for at most the two grace periods.

## The pure core

`mission.py`, `state_machine.py` and `phase_monitor.py` import no ROS and read no
clock — time is passed in. That is what makes the interesting behaviour testable
without a robot: what a hold does to a half-finished step, whether a retry
consumes an attempt, whether a grasp check can fire on the pose the phase started
in. Those are the questions that cost hardware time to answer, so they are
answered in `tests/`.

The split also means a mission file can be validated anywhere:

```bash
python -m cerebel_orchestrator.mission missions/two_station_pick_place.yaml
```

## Frames

```
map ──(static identity, for RViz only)──▶ odom ──(chassis odometry)──▶ base_link
```

There is no localisation. `odom` is the global frame for Nav2 and station poses
are expressed in it, which means they are measured from wherever the wheels were
when the chassis driver came up. [NAVIGATION.md](NAVIGATION.md) covers what that
costs and how to replace it.
