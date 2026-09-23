"""The robot's canonical 16-DOF layout, mirrored from the inference client.

This is the authoritative order used by the recorder, the dataset, the GR00T
checkpoint and ``adibot_gr00t_client``. ``/joint_states`` arrives scrambled and
must be reordered **by name** into this order before any index below means
anything.

Keep this list byte-identical to ``CANONICAL_JOINT_ORDER`` in
``adibot_gr00t_client/inference_client_node.py``. If the two ever disagree, the
orchestrator will read a different joint than the policy commands, and a grasp
check will pass on the wrong finger.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

CANONICAL_JOINT_ORDER: List[str] = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
    "openarm_left_finger_joint1",
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
    "openarm_right_finger_joint1",
]

LEFT_ARM = slice(0, 7)
LEFT_GRIPPER = 7
RIGHT_ARM = slice(8, 15)
RIGHT_GRIPPER = 15

GRIPPER_INDEX = {"left": LEFT_GRIPPER, "right": RIGHT_GRIPPER}
ARM_SLICE = {"left": LEFT_ARM, "right": RIGHT_ARM}

# The recorder wrote gripper actions as normalised_signal * GRIPPER_SCALE, in
# metres. Finger positions read back off /joint_states are in those same metres,
# which is why the grasp thresholds in the params file are metres and not
# radians.
GRIPPER_SCALE = 0.05


def reorder_by_name(
    names: Sequence[str], values: Sequence[float]
) -> Optional[List[float]]:
    """Reorder one ``/joint_states`` field into canonical order.

    Returns None when any canonical joint is missing from the message -- a
    partial state is worse than no state, because the gaps would be read as
    zeros and a zero finger position looks exactly like a closed gripper.
    """
    lookup: Dict[str, float] = {}
    for name, value in zip(names, values):
        lookup[name] = float(value)
    out: List[float] = []
    for name in CANONICAL_JOINT_ORDER:
        if name not in lookup:
            return None
        out.append(lookup[name])
    return out
