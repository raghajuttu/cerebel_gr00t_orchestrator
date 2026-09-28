#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
adibot_gr00t_client.rtc
=======================
Seam alignment for GR00T real-time chunking (RTC). Pure numpy -- no ROS, no
ZMQ -- so it can be unit-tested on any machine (tests/test_rtc.py).

What the server does with what we send
--------------------------------------
With the patched policy server (see server/README.md) a request may carry the
previous action chunk under observation["action"] together with an ``options``
dict. The action head (gr00t/model/gr00t_n1d7/gr00t_n1d7.py,
``get_action_with_features``) then does, in its normalised action space:

    seed = sent[action_horizon - overlap : action_horizon]   # LAST overlap rows
    new[0:overlap]        starts from seed instead of noise
    new[0:frozen]         is held EXACTLY (its denoising velocity is zeroed)
    new[frozen:overlap]   eases in over an exponential ramp (rtc_ramp_rate)
    new[overlap:]         is denoised from noise as usual

The server knows nothing about timing: it always lays the last ``overlap`` rows
it was sent over the first ``overlap`` steps of the new chunk. It is the
client's job to make those rows the steps the arm will be executing at the
instants the new chunk's first steps stand for.

The new chunk is anchored to the observation taken when the request fires. If
the arm is about to execute step ``cursor`` of the current chunk at that
instant, new step j lines up with current step cursor + j, so the seed has to
be prev[cursor : cursor + overlap]. plan_seed() works that out and expresses it
as a row order for the array we send. Sending the previous chunk back
unshifted -- as v0.4.0 did -- is only right when cursor happens to equal
H - overlap, which with prefetch it usually does not.

Before the server can use the seed it re-encodes it the way it encodes a
training label: relative to the NEW observation's state (for relative keys),
min-max normalised, clipped to [-1, 1] and cast to bf16. That round trip is
lossless up to bf16 rounding (a few mrad), which is what freeze_error_mrad()
measures once the new chunk is back: a large error means the seed was ignored.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class SeedPlan:
    """How to turn the previous chunk into the array sent as observation["action"]."""
    idx: np.ndarray   # (horizon,) row of the previous chunk to place at each sent row
    horizon: int      # rows sent; goes out as options["action_horizon"]
    offset: int       # first previous-chunk step the seed covers (== cursor)
    overlap: int      # effective rtc_overlap_steps (<= requested)
    frozen: int       # effective rtc_frozen_steps (<= overlap)


def plan_seed(prev_len: int, cursor: int, overlap: int, frozen: int) -> Optional[SeedPlan]:
    """Align the previous chunk to the request about to be sent.

    Args:
        prev_len: H, number of steps in the previous (current) chunk.
        cursor:   index of that chunk's step the arm executes at the observation
                  instant -- new step 0 lines up with it.
        overlap:  requested rtc_overlap_steps.
        frozen:   requested rtc_frozen_steps.

    Returns None when nothing can be stitched onto: no chunk, or the cursor is
    at or past its end (the plan is used up, e.g. after a long stall).

    The overlap is cut to what remains of the chunk past the cursor, and frozen
    is cut to the overlap. The sent array always has H rows, so that
    options["action_horizon"] equals the chunk length the server itself
    produced; rows before the seed are filled by repeating the earliest
    available step and are never read by the server.
    """
    H, k = int(prev_len), int(cursor)
    if H <= 0 or k < 0 or int(overlap) <= 0:
        return None
    ov = min(int(overlap), H - k)
    if ov <= 0:
        return None
    fr = max(0, min(int(frozen), ov))
    # Row i of the sent array is previous step i + shift, so that the last
    # `ov` rows are exactly prev[k : k + ov]. shift <= 0; indices before the
    # start of the chunk are clamped to step 0.
    shift = k + ov - H
    idx = np.clip(np.arange(H) + shift, 0, H - 1)
    return SeedPlan(idx=idx, horizon=H, offset=k, overlap=ov, frozen=fr)


def build_seed_action(raw_action: dict, plan: SeedPlan) -> dict:
    """Re-index every (B, T, D) array of the server's own action dict along T.

    Keeps the server's key layout and float32 dtype, so what goes back is
    exactly what the server would accept as a training label for this
    embodiment. Raises ValueError for shapes that cannot be re-indexed.
    """
    out = {}
    for key, value in raw_action.items():
        arr = np.asarray(value)
        if arr.ndim != 3:
            raise ValueError(f"action key '{key}' is not (B, T, D): shape {arr.shape}")
        if arr.shape[1] != plan.horizon:
            raise ValueError(
                f"action key '{key}' has {arr.shape[1]} steps, expected {plan.horizon}")
        out[key] = np.ascontiguousarray(arr[:, plan.idx, :], dtype=np.float32)
    return out


def rtc_options(plan: SeedPlan, ramp_rate: float) -> dict:
    """The options dict the patched server / action head expects."""
    return {
        "action_horizon": int(plan.horizon),
        "rtc_overlap_steps": int(plan.overlap),
        "rtc_frozen_steps": int(plan.frozen),
        "rtc_ramp_rate": float(ramp_rate),
    }


def seed_rows(prev_steps: np.ndarray, plan: SeedPlan) -> np.ndarray:
    """The (overlap, D) rows the server lays over new[0:overlap], in canonical
    joint order -- the reference for the freeze check and the chunk store."""
    steps = np.asarray(prev_steps, dtype=np.float32)
    return np.array(steps[plan.offset:plan.offset + plan.overlap], copy=True)


def freeze_error_mrad(new_steps: np.ndarray, seed: np.ndarray, frozen: int) -> float:
    """Largest difference between the new chunk's frozen steps and the seed
    they were supposed to copy, x1000 (mrad for the arm joints). NaN when
    nothing was frozen. A few mrad is bf16 rounding; hundreds means the server
    never applied the seed."""
    n = min(int(frozen), len(new_steps), len(seed))
    if n <= 0:
        return float("nan")
    a = np.asarray(new_steps[:n], dtype=np.float64)
    b = np.asarray(seed[:n], dtype=np.float64)
    return float(np.nanmax(np.abs(a - b))) * 1000.0
