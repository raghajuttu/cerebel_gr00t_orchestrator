"""Named joint-space envelopes, and the check that the arms are inside one.

A ``check_arms`` step asks one question: *are the arms somewhere I am willing to
drive with?* It is the counterweight to ``ends_parked`` — that flag is a claim
about what a policy does, and this is the measurement that either backs it up or
stops the mission before the wheels turn.

An envelope is a set of per-joint ``[min, max]`` bounds, by canonical joint name.
Joints that are not listed are unconstrained, so an envelope can be as loose as
"the shoulders are down and the elbows are in" without pinning a pose that the
policy is entitled to vary — which matters, because the carry pose after a pick
depends on where the object was.

Nothing here imports ROS; the check is a pure function of the 16-DOF canonical
vector.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

from .joints import CANONICAL_JOINT_ORDER


class EnvelopeError(ValueError):
    """An envelope file that cannot be used as written."""


@dataclass(frozen=True)
class Envelope:
    name: str
    description: str
    limits: Dict[str, Tuple[float, float]]

    def check(self, positions: Sequence[float]) -> Optional[str]:
        """None if every constrained joint is inside its bounds, else why not.

        The message names every violation, not just the first: an arm that is
        out of envelope is usually out in several joints at once, and fixing
        them one run at a time is how an afternoon disappears.
        """
        if len(positions) != 16:
            raise ValueError(f"expected 16 canonical joints, got {len(positions)}")
        violations: List[str] = []
        for index, joint in enumerate(CANONICAL_JOINT_ORDER):
            bounds = self.limits.get(joint)
            if bounds is None:
                continue
            low, high = bounds
            value = float(positions[index])
            if value < low or value > high:
                violations.append(f"{joint}={value:+.4f} outside [{low:+.4f}, {high:+.4f}]")
        if not violations:
            return None
        return f"envelope {self.name!r}: " + "; ".join(violations)

    @staticmethod
    def parse(name: str, raw: object) -> "Envelope":
        if not isinstance(raw, dict):
            raise EnvelopeError(f"envelopes.{name}: must be a mapping")
        unknown = set(raw) - {"description", "joints"}
        if unknown:
            raise EnvelopeError(f"envelopes.{name}: unknown keys {sorted(unknown)}")
        joints = raw.get("joints") or {}
        if not isinstance(joints, dict) or not joints:
            raise EnvelopeError(
                f"envelopes.{name}.joints: an envelope that constrains nothing "
                "would pass on any pose at all"
            )
        limits: Dict[str, Tuple[float, float]] = {}
        for joint, bounds in joints.items():
            if joint not in CANONICAL_JOINT_ORDER:
                raise EnvelopeError(
                    f"envelopes.{name}.joints: {joint!r} is not a canonical joint name"
                )
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                raise EnvelopeError(
                    f"envelopes.{name}.joints.{joint}: must be [min, max]"
                )
            low, high = float(bounds[0]), float(bounds[1])
            if not low < high:
                raise EnvelopeError(
                    f"envelopes.{name}.joints.{joint}: min {low} is not below max {high}"
                )
            limits[joint] = (low, high)
        return Envelope(
            name=name,
            description=str(raw.get("description", "")),
            limits=limits,
        )


def load_envelopes(path: str) -> Dict[str, Envelope]:
    """Read an envelope file. An empty path means no envelopes are defined."""
    if not path:
        return {}
    full = os.path.expanduser(path)
    with open(full, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    body = raw.get("envelopes", raw)
    if not isinstance(body, dict) or not body:
        raise EnvelopeError(f"{path}: no envelopes found")
    return {name: Envelope.parse(name, value) for name, value in body.items()}
