# Client task control — one client, prompts switched at runtime

A feature of this repository's copy of the inference client
(`cerebel_orchestrator/gr00t_client/`, version `1.0.0+cerebel.1`; where it
came from: `gr00t_client/PROVENANCE.md`). The orchestrator uses it for
`policy_switching: prompt` — see [POLICY_SWITCHING.md](POLICY_SWITCHING.md).

`task_control:=true`. Off by default: without it the node runs
`task_description` from startup until it is killed, exactly as groot_deployment
1.0.0 does.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `task_control` | bool | `false` | `true` = start **idle** (connected, pinged, publishing nothing) and take the task from `task_command_topic`; `task_description` is then ignored. Requires `prefetch_enable:=true` — the node refuses to start otherwise |
| `task_command_topic` | string | `~/task` | `std_msgs/String`, JSON `{"epoch": int, "task": str}`. Non-empty task = switch to it (or start); empty = pause. Commands at or below the current epoch are ignored. RELIABLE + TRANSIENT_LOCAL |
| `task_state_topic` | string | `~/task_state` | `std_msgs/String`, JSON `{"state": "idle"\|"switching"\|"active", "epoch", "task", "tick", "t_unix"}`. `active` is published on the tick the new task's first chunk starts executing. RELIABLE + TRANSIENT_LOCAL |

The node is named `gr00t_client`, so `~/task` is `/gr00t_client/task` — the
orchestrator's `client_task_topic` default.

## Why

Switching between prompts of **one checkpoint** used to mean stopping this
node and starting another with a different `task_description`. On Adibot that
left the arms uncommanded for the whole restart — about 0.27 s of shutdown and
1.0 s of startup warm, 3.8 s on the first switch of a run (measured
2026-09-28) — while the controller latched the last pose.

Almost all of that is process startup: the `ros2 run` CLI, the imports, DDS
discovery, the ping. None of it is needed to change the prompt, because the
prompt is only ever read when an observation is built: every request carries
`task_description` as it stands at that moment.

## How it behaves

The node starts **idle**: connected, pinged, control loop running, publishing
nothing. It takes commands on `task_command_topic`:

```json
{"epoch": 3, "task": "scan and place the object"}   // switch, or start
{"epoch": 4, "task": ""}                             // pause
```

**Switch (active → new prompt).** The current buffer keeps executing; the next
request goes out on the next tick with the new prompt instead of waiting for
`prefetch_lead`. With RTC on, that request is seeded from the current plan, so
the new prompt's chunk grows out of the motion in progress and the arm does not
stop at the switch. Measured in the test harness (`tests/test_client_node_loop.py`, `TaskControlTest`): a step executed on every tick
across the switch and a continuous trajectory through it. The switch is
complete after one round trip, plus the remainder of an old-prompt request if
one was already in flight when the command arrived.

**An old-prompt response is discarded.** Every request carries the epoch it
was built under; a response from an older epoch is dropped on arrival and
counted as `superseded_chunks` in the sidecar. The ZMQ REQ socket cannot cancel
a request, so the worker is left to finish it.

**Pause.** Stops commanding on the next tick: the buffer and the RTC plan are
dropped and the controller latches the last command, so the arm holds. A
resume after a pause starts clean — the first request is unseeded, because
seeding from a plan the arm stopped following would stitch onto motion that is
not happening.

**Epochs only go up.** A command at or below the current epoch is ignored. The
command topic is TRANSIENT_LOCAL (a command sent before this node subscribed is
still delivered), so replays are expected and harmless.

## State

`task_state_topic` reports `idle` (at startup after the ping, and on a pause),
`switching` (command taken) and `active` (the tick the new task's first chunk
starts executing — the moment the switch is complete as the arm sees it). The
node also logs that moment with the time since the command.

## Logging

One run log for the whole session instead of one per phase. The sidecar gains
`task_events` (every start/switch/pause with its epoch, tick and time) and
`superseded_chunks`, and records `task_control`. The per-tick CSV is unchanged;
`task_events` ticks map it back to phases.

## What is not validated yet

The RTC seam across two **different prompts**. Within one prompt the seed and
the new chunk agree by construction; across a switch the frozen steps are the
old prompt's plan and the rest is denoised under the new prompt. That is the
intended blend, but how it behaves on hardware — especially the first second of
a place that starts mid-motion rather than from the pause the demonstrations
recorded — has to be watched on the robot before it is trusted.
