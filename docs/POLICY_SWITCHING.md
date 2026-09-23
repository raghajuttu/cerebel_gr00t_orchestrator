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

On the GPU box that looks like:

```bash
# terminal 1 — the pick checkpoint
python scripts/inference_service.py --server --port 5555 \
    --model_path /path/to/checkpoint-pick   ...
# terminal 2 — the place checkpoint
python scripts/inference_service.py --server --port 5556 \
    --model_path /path/to/checkpoint-place  ...
```

and through an SSH forward, one `-L` per port:

```bash
autossh -M 0 -N \
    -L 5555:127.0.0.1:5555 \
    -L 5556:127.0.0.1:5556 \
    -o "ServerAliveInterval 30" -o "ExitOnForwardFailure yes" \
    -p 3129 adibot@<gpu-box>
```

The forward destination must be `127.0.0.1`, not `localhost` — on a dual-stack
host `localhost` can resolve to `::1` while the server binds IPv4 `0.0.0.0`, and
the symptom is a silent ping timeout at client startup. This is recorded in the
deployment notes for the single-policy case and applies identically per port here.

The orchestrator prints the grouping at startup so a mistake is visible before
anything moves:

```
server 127.0.0.1:5555     : pick_cube, place_cube (ONE checkpoint, several prompts)
server 127.0.0.1:5556     : handoff
```

If you meant two checkpoints and see one line, the second server is not up or the
port in the mission is wrong.

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
