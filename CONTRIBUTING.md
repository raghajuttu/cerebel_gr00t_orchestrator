# Contributing

Same rules as `adibot_gr00t_client`, because the two are deployed together.

* **One logical change per commit**, with a `feat:` / `fix:` / `docs:` / `chore:`
  prefix and a message that says what was wrong, not just what changed.
* **Unit tests green**: `python -m pytest tests -q`. The pure modules
  (`mission.py`, `state_machine.py`, `phase_monitor.py`, and
  `policy_runner.build_argv`) import no ROS and must stay that way — that is what
  makes them testable, and it is where the behaviour that costs hardware time to
  debug lives.
* **A `CHANGELOG.md` entry** that names the issue.
* **A `docs/VALIDATION.md` entry for anything that touches the control path** —
  the step handlers, the adapter, the parker, the phase monitor. What ran, on
  what, with which parameters, and what was observed.
* **New placeholder numbers go in `docs/TUNING.md`** with how to measure the real
  one. A magic number with no measurement path is a bug waiting for hardware.
* **Keep `joints.py` byte-identical to the inference client's
  `CANONICAL_JOINT_ORDER`.** If the two ever disagree, this package reads a
  different joint than the policy commands, and a grasp check passes on the wrong
  finger.
