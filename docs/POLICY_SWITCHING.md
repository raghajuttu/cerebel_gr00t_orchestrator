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

Each entry is everything needed to start one `adibot_gr00t_client` process. The
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
ros2 run adibot_gr00t_client inference_client --ros-args \
  -p task_description:='pick up the green cube and place it in the box' \
  -p server_host:='127.0.0.1' -p server_port:=5555 \
  -p log_run_name:='two_station_pick_place_c1_s3_run_policy_pick_cube' \
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
  `<mission>_c<cycle>_s<step>_<kind>_<policy>[_a<attempt>]`. The per-tick CSV, the
  sidecar and the chunk store all carry it, so a run of a four-phase mission leaves
  four self-describing sets of logs that
  [`adibot_run_browser`](https://github.com/raghajuttu/adibot_run_browser) opens
  directly.
* **`params` cannot override the policy identity.** `mission.py` refuses a
  `params` block containing `task_description`, `server_host`, `server_port` or
  `checkpoint_label` — otherwise a mission's logs would claim one policy while
  another ran.
* **The command itself is configurable.** `policy_cmd` in `params/orchestrator.yaml`
  defaults to `["ros2", "run", "adibot_gr00t_client", "inference_client"]`. If the
  orchestrator is started from an environment that has not sourced the workspace,
  wrap it:
  `["bash", "-lc", "source ~/Desktop/gripper/install/setup.bash && exec ros2 run adibot_gr00t_client inference_client"]`.

## Stopping

`SIGINT` first — the same signal Ctrl-C sends, so rclpy shuts the node down
through its normal path. Then `SIGTERM` after `sigint_grace_s` (5 s), then
`SIGKILL` after `sigterm_grace_s` (3 s). The signal goes to the process *group*,
so anything the client spawned goes with it.

After the client is gone the arms hold their last commanded pose, powered. Nothing
relaxes them and nothing moves them back. The next step in a well-formed mission
is `park_arms`.

## Two things this does not do

**It does not blend phases.** Each policy phase starts from whatever pose the
previous one left, with a fresh client and a fresh ZMQ connection. There is no
continuity of the action chunk across a switch — RTC works *within* a session, not
across two. If a pick and a place need to flow into each other without a pause,
that is one policy with one prompt, not two phases.

**It does not choose a policy.** Which policy runs where is written in the mission
file by a human. There is no perception step that decides "this is a cube, use the
cube policy". A mission is a fixed sequence; the only runtime branching is
retry-on-failure.
