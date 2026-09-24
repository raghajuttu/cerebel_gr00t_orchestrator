#!/usr/bin/env python3
"""What happened to the arms at a policy switch, in numbers.

"The arms jerked a bit" is a feeling. This turns it into four measurements,
taken from the per-tick CSVs that ``adibot_gr00t_client`` writes -- one per
policy phase, named after the mission step.

    python3 tools/seam_report.py ~/adibot_logs

It finds the two most recent run logs, treats the first as the phase that ended
and the second as the phase that began, and reports the seam between them.

## The four numbers, and what each one blames

**Drift during the gap** -- how far the arm actually moved between the last
sample of the old phase and the first sample of the new one, while nobody was
commanding it. Should be near zero: ``forward_position_controller`` latches its
last command. If it is not, the latch is not holding and the fix is
``hold_arms_between_policies: true``, which republishes the last commanded pose
at 30 Hz through the gap.

**Settling** -- the standing offset between the last command and where the arm
actually was. This robot's follower sits behind its command and never catches
up, so when the commands stop the arm keeps moving a little as it closes that
gap. Expected to be a few hundredths of a radian. It is not a fault, but it
does mean the new policy's first observation is not quite the old policy's last
command.

**The command step** -- the distance from the old phase's last commanded pose
to the new phase's first commanded pose. THIS IS THE JERK. Everything else is
context for it.

**Where the step came from** -- the same distance measured from the arm's
actual position when the new phase started. If the command step is large but
this is small, the new policy simply continued from where the arm was, and the
jerk was the arm catching up to a stale command. If both are large, the new
policy wanted to be somewhere else, and no amount of holding will fix it --
that is a start-state mismatch between the two behaviours.
"""

from __future__ import annotations

import csv
import glob
import os
import sys
from typing import Dict, List, Optional, Tuple

JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)] + ["openarm_left_finger_joint1"] + \
         [f"openarm_right_joint{i}" for i in range(1, 8)] + ["openarm_right_finger_joint1"]
ARM = [j for j in JOINTS if "finger" not in j]


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def vector(row: Dict[str, str], prefix: str, joints: List[str]) -> Optional[List[float]]:
    try:
        return [float(row[f"{prefix}{j}"]) for j in joints]
    except (KeyError, ValueError):
        return None


def spread(a: List[float], b: List[float], joints: List[str]) -> Tuple[float, float, str]:
    """(max, mean, which joint) of |a - b|."""
    diffs = [abs(x - y) for x, y in zip(a, b)]
    worst = max(range(len(diffs)), key=lambda i: diffs[i])
    return max(diffs), sum(diffs) / len(diffs), joints[worst]


def describe(label: str, a, b, joints: List[str]) -> None:
    if a is None or b is None:
        print(f"  {label:34s} -- not in the log")
        return
    hi, mean, worst = spread(a, b, joints)
    flag = ""
    if hi > 0.20:
        flag = "   <-- large"
    elif hi > 0.05:
        flag = "   <-- worth a look"
    print(f"  {label:34s} max {hi:6.3f} rad  mean {mean:6.3f}  worst {worst}{flag}")


def main(argv: List[str]) -> int:
    log_dir = os.path.expanduser(argv[1] if len(argv) > 1 else "~/adibot_logs")
    runs = sorted(glob.glob(os.path.join(log_dir, "*.csv")), key=os.path.getmtime)
    if len(runs) < 2:
        print(f"need two run CSVs in {log_dir}; found {len(runs)}")
        return 1

    before, after = runs[-2], runs[-1]
    rows_before, rows_after = read_rows(before), read_rows(after)
    if not rows_before or not rows_after:
        print("one of the logs has no rows -- did that phase actually run?")
        return 1

    last, first = rows_before[-1], rows_after[0]
    gap = float(first["wall_time"]) - float(last["wall_time"])

    print(f"phase that ended : {os.path.basename(before)}  ({len(rows_before)} ticks)")
    print(f"phase that began : {os.path.basename(after)}  ({len(rows_after)} ticks)")
    print(f"gap between them : {gap:.2f} s with nobody commanding the arms")
    print()

    cmd_last = vector(last, "cmd_", ARM)
    pos_last = vector(last, "actual_pos_", ARM)
    cmd_first = vector(first, "cmd_", ARM)
    pos_first = vector(first, "actual_pos_", ARM)

    describe("drift during the gap", pos_last, pos_first, ARM)
    describe("settling (command vs actual)", cmd_last, pos_last, ARM)
    describe("THE COMMAND STEP (the jerk)", cmd_last, cmd_first, ARM)
    describe("step from where the arm was", pos_first, cmd_first, ARM)

    print()
    for side, index in (("left", 7), ("right", 15)):
        name = JOINTS[index]
        try:
            a, b = float(last[f"actual_pos_{name}"]), float(first[f"actual_pos_{name}"])
            print(f"  {side} finger across the gap    {a:.4f} -> {b:.4f}"
                  f"{'   <-- CHANGED, the grip moved' if abs(a - b) > 0.004 else ''}")
        except (KeyError, ValueError):
            pass

    print()
    print("Reading it: a large command step with a small step-from-where-the-arm-was")
    print("means the arm was catching up to a stale command -- turn on")
    print("hold_arms_between_policies. Both large means the two policies disagree")
    print("about where this phase starts, which holding cannot fix.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
