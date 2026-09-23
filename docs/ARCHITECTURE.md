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
phase        base            arms                          who commands the arms
----------   -------------   ---------------------------   ---------------------
navigate     Nav2            holding a known pose          nobody
run_policy   gate shut       policy                        the child process
park_arms    gate shut       ramp                          the orchestrator
check_arms   gate shut       holding, being verified       nobody
wait         gate shut       holding                       nobody
hold/estop   gate shut       holding                       nobody
```

"Holding" is not a null state. `forward_position_controller` latches its last
command, so after the inference client exits the arms stay exactly where the last
action chunk put them, powered and stiff.

That latching is what makes pick-and-carry work: the pick policy ends holding the
object, the client is stopped, and the arms keep holding it through the drive. It
is also what makes an unguarded `navigate` dangerous, because "wherever the last
action chunk put them" is only safe if somebody established what that is. A
navigate is therefore preceded by one of three things — `park_arms`, `check_arms`,
or a `run_policy` marked `ends_parked: true`. `mission.py` lints for it and the
orchestrator prints the warnings at startup.

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

## The two loops

The node runs two loops at two rates, plus the sensor callback that feeds them:

| Loop | Rate | Job | Touches |
|---|---|---|---|
| **execution** — `_execution_tick` | `control_rate_hz`, 30 Hz | sustain what is currently running | publishes the park ramp and the `base_enable` heartbeat |
| **supervision** — `_supervisor_tick` | `supervisor_rate_hz`, 10 Hz | is this step done? if so, switch | starts and stops steps; commands no actuator directly |
| **observation** — `_on_joint_states` | whatever the robot publishes, ~30 Hz | fold each sample into the completion check | nothing; it only accumulates |

The split is along *what each one is allowed to do*, and that is what lets the
rates be independent:

* The execution loop decides nothing. Raising it makes the park ramp smoother and
  the heartbeat more robust, and changes no behaviour.
* The supervision loop commands nothing. Raising it makes the robot switch steps
  sooner after a condition is met, and makes no motion smoother.

The execution loop is mostly idle, because for two of the four step kinds the
thing doing the work has its own loop: Nav2 runs its controller at 20 Hz and the
inference client its control loop at 30 Hz. The orchestrator starts them and gets
out of the way. What is left for this loop is the park ramp — the one motion the
orchestrator produces itself — and the heartbeat, which wants a steady fast rate
so the base's fail-closed timeout can be short.

**Both loops run on the same single-threaded executor.** They never interleave,
so no state is shared across threads and there are no locks anywhere in this
package. Two rates, one thread: the separation is in responsibilities, not in
concurrency, and adding a `MultiThreadedExecutor` here would buy nothing and cost
every race that currently cannot happen.

### Why observation is a third thing

Completion is judged from `/joint_states`, and those samples are folded in by the
subscription rather than sampled by either loop:

```python
def _on_joint_states(self, msg):          # ~30 Hz, the robot's rate
    self._monitor.observe(now, ordered)   # accumulate; decide nothing

def _supervisor_tick(self):               # 10 Hz, our rate
    verdict = self._monitor.verdict(now)  # decide; observe nothing
```

This is not tidiness. `PhaseMonitor` judges "the arm has stopped" from a joint
**speed**, and a speed needs the interval between two samples. If the supervisor
sampled the latest pose on its own tick instead, the threshold would really mean
"how far a joint moved between two of my ticks" — a number whose meaning changes
whenever either rate changes, and which cannot be transferred from one robot or
one run to another. Tuned at 10 Hz, the same value silently means something three
times stricter at 30 Hz. `tests/test_two_loops.py` pins the independence across
sensor rates of 10/30/100 Hz and supervisor rates of 5/10/50 Hz.

The same split exists in `ArmParker` (`step` publishes, `outcome` decides) for the
same reason: the ramp's smoothness should not be a side effect of how often the
mission logic is checked.

## Why nothing blocks in the node

Every handler is poll-shaped: `send` then `poll`, never `wait`. That is what keeps
the e-stop subscription and the six service calls live while a 90-second policy
phase is running, and it is why the supervision loop can be a plain timer rather
than a thread per step. `PolicyRunner.stop` is the one place that blocks, and it
blocks for at most the two signal grace periods.

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
