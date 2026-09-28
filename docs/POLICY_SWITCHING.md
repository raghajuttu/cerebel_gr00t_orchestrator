# Switching policies

## What a "policy" is here

A mission's `policies:` table holds named sessions, not models:

```yaml
policies:
  pick_cube:
    task_description: "pick up the green cube and place it in the box"
    server_host: 127.0.0.1
    server_port: 5555
    checkpoint_label: "adibot-green-pick"
    params:
      execution_horizon: 16
      prefetch_enable: true
      rtc_enable: false
      enable_limits: false
```

Each entry is everything needed to start one inference client process (this
repository's vendored copy, `cerebel_orchestrator/gr00t_client/`). The
`params` block is passed straight through as ROS parameters, so anything in that
package's `docs/ARGUMENTS.md` is settable per phase — the RTC knobs, the execution
horizon, the safety limits, the topic names, which arms are enabled.

## The two kinds of switch

| Switch | How | Cost |
|---|---|---|
| **Same checkpoint, different prompt** | two policies, same `server_host:server_port`, different `task_description` | one client restart, a few seconds |
| **Different checkpoint** | two policies, different ports, one policy server per checkpoint on the GPU box | the same restart; the servers were already running |

The policy server serves exactly one checkpoint for its lifetime. **Nothing in
this package can make a running server load a different checkpoint** — if a
mission needs two checkpoints, two servers have to be up, on two ports, before the
mission starts.

The orchestrator prints the grouping at startup so a mistake is visible before
anything moves:

```
server 127.0.0.1:5555     : pick_cube, place_cube (ONE checkpoint, several prompts)
```

### The current setup: one checkpoint on cthor

One fine-tune, one server, one port, reached through the SSH forward from the
robot computer:

```bash
# on cthor (inside the container, with the repo copy first on the path)
PYTHONPATH=/workspace/repo python scripts/inference_service.py \
    --server --port 5555 --model_path /path/to/checkpoint ...

# on the robot computer -- one -L, destination 127.0.0.1, NOT localhost
autossh -M 0 -N -L 5555:127.0.0.1:5555 \
    -o "ServerAliveInterval 30" -o "ExitOnForwardFailure yes" \
    -p 3129 adibot@<cthor>
```

`127.0.0.1` rather than `localhost` matters: on a dual-stack host `localhost` can
resolve to `::1` while the server binds IPv4 `0.0.0.0`, and the symptom is a
silent ping timeout at client startup rather than an error. `ServerAliveInterval`
is what stops an idle forward being dropped mid-mission.

Both mission policies therefore carry `server_host: 127.0.0.1, server_port: 5555`,
and the switch between them is a client restart with a different
`task_description` — no ports change, nothing on cthor is touched.

**Check before believing that two prompts do anything.** A GR00T policy is
conditioned on the language annotation it was fine-tuned with. If the dataset
carried one annotation for every episode:

```bash
wc -l <dataset>/meta/tasks.jsonl     # one line -> one annotation
```

then a second, different prompt is a string the checkpoint has never seen. What
comes back is undefined behaviour, not "the place half of the task", and it will
look like a policy that suddenly got worse. With a single-annotation checkpoint,
set both `task_description`s to that annotation verbatim and let the two phases
differ by their **`until` conditions** instead — the pick phase ends on
`grasp: closed`, the place phase on `grasp: open`. Same policy, same prompt, two
differently-terminated runs. That is what the shipped mission does.

Distinct prompts start being worth something once the dataset has distinct
annotations per segment, which is a data-collection change rather than a
configuration one.

### Preflight

The orchestrator pings every distinct server once, at mission start, before
anything moves (`policy_preflight`, on by default). If the ping fails the mission
faults immediately instead of driving to the pick station and discovering it
there.

This is worth more than it sounds over a forward. `127.0.0.1:5555` accepts a TCP
connection whenever the local `autossh` process is alive, so `ss -tlnp` says yes
even when cthor is unreachable or the server has died. Only a round trip proves
the far end is there. The same check by hand:

```bash
ros2 run cerebel_orchestrator ping_policy 127.0.0.1:5555
```

## What the orchestrator runs

Exactly this, per phase, with the phase's own values:

```bash
ros2 run cerebel_orchestrator inference_client --ros-args \
  -p task_description:='pick up the green cube and place it in the box' \
  -p server_host:='127.0.0.1' -p server_port:=5555 \
  -p log_run_name:='two_station_pick_place_c1_s3_run_policy_pick_cube_20260928_174335' \
  -p log_dir:='~/adibot_logs' \
  -p checkpoint_label:='adibot-green-pick' \
  -p execution_horizon:=16 -p prefetch_enable:=true -p rtc_enable:=false \
  -p enable_limits:=false
```

It is printed in full at startup for every policy in the mission (with
`RUNLABEL` in place of the run name), so you can copy one and run it by hand to
compare. Things worth noting:

* **Strings are always YAML-quoted.** The value half of `-p name:=value` is parsed
  as YAML, so an unquoted `5555` would arrive as an integer and an unquoted task
  description would lose to whitespace. `task_description` must reach the policy
  byte-identical to the training annotation or the policy is being prompted with
  something it never saw.
* **`log_run_name` identifies the mission step**:
  `<mission>_c<cycle>_s<step>_<kind>_<policy>[_a<attempt>]_<YYYYmmdd_HHMMSS>`, the
  stamp being when the mission started, so a run never overwrites an earlier
  one's logs. The per-tick CSV, the
  sidecar and the chunk store all carry it, so a run of a four-phase mission leaves
  four self-describing sets of logs that
  [`adibot_run_browser`](https://github.com/raghajuttu/adibot_run_browser) opens
  directly.
* **`params` cannot override the policy identity.** `mission.py` refuses a
  `params` block containing `task_description`, `server_host`, `server_port` or
  `checkpoint_label` — otherwise a mission's logs would claim one policy while
  another ran.
* **The command itself is configurable.** `policy_cmd` in `params/orchestrator.yaml`
  defaults to `["ros2", "run", "cerebel_orchestrator", "inference_client"]` — the
  vendored client. Point it at `adibot_gr00t_client` to run groot_deployment's
  copy instead (process switching only; it has no task control). If the
  orchestrator is started from an environment that has not sourced the workspace,
  wrap it:
  `["bash", "-lc", "source ~/Desktop/effort/install/setup.bash && exec ros2 run cerebel_orchestrator inference_client"]`.

## Re-initialising the arm controller between policies

The manual procedure on this robot is: bring the forward position controller
up, run one VLA, bring it up again, run the next. The orchestrator switches
policies by restarting the inference client, so it has to do that same step.

`policy_prepare_cmd` is where it goes. It runs once, immediately before each
client starts:

```yaml
policy_prepare_cmd: ["ros2", "control", "switch_controllers",
                     "--activate", "left_forward_position_controller",
                     "--activate", "right_forward_position_controller"]
policy_prepare_timeout_s: 15.0
policy_prepare_settle_s: 0.0
```

It is **empty by default**, so a robot that does not need it behaves exactly as
before. The command is a list, not a shell string: there is no shell, so no
quoting to get wrong, and no `&&`. Wrap anything more involved in a script.

Ordering within one phase:

```
prepare_cmd  ->  settle  ->  client starts  ->  server ping  ->  first chunk
```

A non-zero exit, a missing binary or a timeout fails the phase **before** any
client is started, with the command's last output lines in the reason. Nothing
has moved at that point.

### The thing to check before trusting it

Whatever this command does, it must not disturb what the arms are holding.

Pick-and-carry depends on `forward_position_controller` latching its last
command across the switch: the pick policy ends holding the object, the client
is killed, and the arms keep holding it through the base move and into the
place policy. If re-activating the controller re-seeds that latch from the
current measured state, the standing follower offset gets baked in at every
switch. If it drops the latch entirely, the object falls.

Test it directly, before any mission: pick something up by hand into the
gripper, run the prepare command, and watch whether the arm holds, sags or
lets go. That answer decides whether the carry steps in
`three_station_kit.yaml` are sound or whether the place has to happen at the
same station as the pick.

## Stopping

`SIGINT` first — the same signal Ctrl-C sends, so rclpy shuts the node down
through its normal path. Then `SIGTERM` after `sigint_grace_s` (5 s), then
`SIGKILL` after `sigterm_grace_s` (3 s). The signal goes to the process *group*,
so anything the client spawned goes with it.

After the client is gone the arms hold their last commanded pose, powered. Nothing
relaxes them and nothing moves them back. The next step in a well-formed mission
is `park_arms`.

## Prompt switching: one client for the mission

Everything above is `policy_switching: process`, the default and the validated
path. Measured on Adibot on 2026-09-28 it costs 0.27 s of shutdown and about
1.0 s of startup warm (3.8 s on a run's first switch), with nothing commanding
the arms in between. Almost all of that is process startup, and none of it is
needed when the two phases are prompts of **one checkpoint**, because the client
reads its prompt afresh for every request.

`policy_switching: prompt` runs one client (the vendored client with
`task_control:=true`, [CLIENT_TASK_CONTROL.md](CLIENT_TASK_CONTROL.md)) for the
whole mission and switches its prompt instead:

```yaml
policy_switching: prompt     # at the top level of the mission
prompt_blend: true           # default
```

or, to A/B the same mission without editing it:

```bash
ros2 launch cerebel_orchestrator orchestrator.launch.py mission:=pick_then_place \
    use_nav2:=false policy_switching:=prompt prompt_handover:=pause
```

What changes:

* **The client starts with the mission**, before step 0, and the first phase
  waits for it to report ready. The cold start is paid once, outside any phase.
* **Policy -> policy** is a prompt switch. With `prompt_blend: true` the old
  prompt's plan keeps executing while the new prompt's first chunk is requested,
  and RTC seeds that chunk from the plan — the arm does not stop. With
  `prompt_handover:=pause` (or `prompt_blend: false`) the old prompt is paused at
  the phase end and the new one starts from rest one round trip later, the way
  the demonstrations began.
* **Policy -> anything else** pauses the client at the phase end, and the next
  step does not start until the client has *acknowledged* the pause. A client
  that has not acknowledged within `prompt_pause_timeout_s` (1 s; it normally
  takes one 33 ms tick) is killed, and the step goes ahead. This is what keeps a
  base move from starting while a policy could still be commanding the arms.
* **E-stop, hold, abort and the end of the mission** kill the client exactly as
  process switching does. The next policy phase starts a fresh one.
* A client that dies between phases is restarted at the next policy phase; one
  that dies during a phase fails that phase, as before.
* **One run log** for the mission (`<mission>_prompt_client_<YYYYmmdd_HHMMSS>`), not one per phase.
  Its sidecar lists every switch in `task_events`.

The mission is refused at load if its policies differ in anything but
`task_description` — another server, checkpoint label or client parameter can
only be reached by a new client.

The log reports each switch as the time from the command to the new prompt's
first chunk executing, and from the end of the previous phase.

**Not validated on hardware yet:** the RTC seam across two different prompts.
Start with `prompt_handover:=pause`, which never blends, then try `blend` with a
hand on the e-stop and watch the first second of the place.

## Two things this does not do

**Process switching does not blend phases.** Each policy phase starts from
whatever pose the previous one left, with a fresh client and a fresh ZMQ
connection, so RTC cannot carry across the switch. Prompt switching with
`prompt_blend: true` is the mode that does.

**It does not choose a policy.** Which policy runs where is written in the mission
file by a human. There is no perception step that decides "this is a cube, use the
cube policy". A mission is a fixed sequence; the only runtime branching is
retry-on-failure.
