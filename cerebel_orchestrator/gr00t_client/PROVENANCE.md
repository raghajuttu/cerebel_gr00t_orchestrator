# Where this client came from

The GR00T inference client, copied into the orchestrator so the orchestrator
does not depend on another repository at runtime.

**Source:** `raghajuttu/groot_deployment`, package `adibot_gr00t_client`,
commit `d576ed9` (v1.0.0, 2026-09-04) — the version that ran on Adibot.

**Copied verbatim** in the commit that added this file. MD5 of the files as
copied, identical to the source at that commit:

| File | MD5 |
|---|---|
| `__init__.py` | `d41d8cd98f00b204e9800998ecf8427e` |
| `inference_client_node.py` | `d61e33c39c6fcec83db049189e5b0721` |
| `rtc.py` | `1575951c48cceaf1d710f8ee422f2487` |
| `data_logger.py` | `55d175ea5e1375f39574debda7e5d56c` |

Not copied: the policy-server patch (`server/`, it runs on the GPU machine, not
the robot), `scripts/`, `tools/` and the server probes.

**Every change since is its own commit.** To see exactly how this copy differs
from the validated client:

```bash
git log --oneline -- cerebel_orchestrator/gr00t_client/
git diff <the commit that added this file> -- cerebel_orchestrator/gr00t_client/
```

**This is a fork, not a mirror.** A fix made in `groot_deployment` does not
reach this copy unless it is ported by hand, and the other way round.
