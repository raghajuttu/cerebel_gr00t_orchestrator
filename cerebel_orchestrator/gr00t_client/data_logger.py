#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
adibot_gr00t_client.data_logger
=================================

Run logging for the GR00T inference client.

Each run writes THREE files into <log_dir>, sharing the run name:

  <run_name>.csv         one row per control tick -- every EXECUTED step
  <run_name>.meta.json   the run's configuration, the server's first info
                         dict, and outcome counts stamped at close
  <run_name>.chunks.npz  every action chunk the server returned, in full

The CSV pairs what the robot actually was (the reordered /joint_states
position, plus velocity/effort when the driver provides them) against what we
commanded on that tick (the VLA's 16-D action step). That is the file you diff
to see tracking error.

`latency_ms` is filled only on the tick where a new server response arrives
(horizon_idx 0); it is blank on the reused steps in between. `inference_seq`
and `horizon_idx` let you group rows back to the server response that produced
them.

The chunk store exists because the CSV only ever holds executed steps: the
served checkpoint can predict 40 steps while execution_horizon executes 16, so
more than half of every prediction would otherwise be discarded unrecorded.
Chunks are recorded before the accept/drop decision, so even one dropped as
entirely stale is kept. Since v0.7.0 each chunk also carries the facts of the
request that produced it -- when it was sent, the joint state it saw, and, with
RTC on, the seed rows sent back, the effective overlap/frozen/ramp, the chunk
they came from, and how far the returned frozen block strayed from that seed.

The CSV is line-buffered, so a Ctrl-C loses at most the row in flight. The
chunk store is rewritten by a daemon thread every CHUNK_FLUSH_EVERY chunks and
again at close; a hard kill loses at most that many chunks, never the CSV.
"""

import json
import os
import threading
import time
from datetime import datetime
from typing import Any, Optional, Sequence

import numpy as np


# Ask the writer thread to persist the chunk store every N chunks (and
# always at close). The write itself never happens on the caller's thread:
# rewriting the whole store is O(number of chunks so far), and the caller here
# is the robot's 30 Hz control loop.
CHUNK_FLUSH_EVERY = 25


def _int(v, default: int) -> int:
    """int(v), or `default` when v is None."""
    return default if v is None else int(v)


def _float(v) -> float:
    """float(v), or NaN when v is None."""
    return float("nan") if v is None else float(v)


def _fmt(v) -> str:
    """Compact fixed-precision float; None/NaN -> '' (blank cell)."""
    if v is None:
        return ""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return ""
    if np.isnan(f):
        return ""
    return f"{f:.9g}"


class InferenceLogger:
    """Writes a run's CSV, its configuration sidecar and its chunk store."""

    def __init__(self,
                 log_dir: str,
                 joint_names: Sequence[str],
                 run_name: Optional[str] = None,
                 ros_logger=None):
        self.joint_names = list(joint_names)
        self.ros_logger = ros_logger

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = run_name or f"run_{stamp}"

        log_dir = os.path.expanduser(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f"{run_name}.csv")
        # Sidecar holding the run's configuration. Kept OUT of the CSV: these
        # values are constant for the whole run, so a column per parameter
        # would repeat itself on every tick and pollute the per-tick schema.
        self.meta_path = os.path.join(log_dir, f"{run_name}.meta.json")
        self._meta: dict[str, Any] = {}
        # Full action chunks (executed AND unexecuted steps), written to a
        # third file beside the CSV. ~2.5 KB per 40x16 chunk, so a long run is
        # a couple of MB. Kept in memory and flushed periodically; a crash
        # costs at most CHUNK_FLUSH_EVERY chunks, never the CSV.
        self.chunks_path = os.path.join(log_dir, f"{run_name}.chunks.npz")
        self._chunk_seq: list[int] = []
        self._chunk_t: list[float] = []
        self._chunk_skip: list[int] = []
        self._chunk_data: list[np.ndarray] = []
        # Per-chunk request facts (v0.7.0), parallel to the lists above.
        self._chunk_t_req: list[float] = []            # NaN when unknown
        self._chunk_req_tick: list[int] = []           # -1 when unknown
        self._chunk_obs_state: list[Optional[np.ndarray]] = []
        self._chunk_rtc: list[Optional[dict]] = []     # None: no seed was sent
        self._chunks_since_flush = 0
        # The control thread only ever appends under this lock and sets the
        # event; a daemon writer does the actual file write.
        self._chunk_lock = threading.Lock()
        self._chunk_wake = threading.Event()
        self._chunk_writer: Optional[threading.Thread] = None
        self._chunk_stop = False

        # Origin for the relative timestamp column.
        self.t0 = time.time()

        self._f = open(self.csv_path, "w", buffering=1)
        self._f.write(",".join(self._header()) + "\n")

        self.n_steps = 0
        self._closed = False

    # ---------------------------------------------------------------- header
    def _header(self):
        # chunk_len / skip_steps / rtc_applied are per-CHUNK facts, written on
        # that chunk's first executed tick and blank elsewhere (same convention
        # as latency_ms). None can be recovered from the other columns:
        #   chunk_len   - what the server sent; max(horizon_idx)+1 only shows
        #                 what was EXECUTED, and prefetch truncates chunks.
        #   skip_steps  - the skip actually applied on arrival.
        #   rtc_applied - whether that request carried a previous chunk. The
        #                 first request of a run never does, and a reconnect
        #                 can drop it, so it is not implied by the params.
        # buffer_len is per tick: starvation and whether prefetch_lead is
        # behaving become directly visible instead of inferred from time gaps.
        cols = ["wall_time", "t_rel", "tick", "inference_seq",
                "horizon_idx", "latency_ms",
                "chunk_len", "skip_steps", "rtc_applied", "buffer_len"]
        cols += [f"actual_pos_{n}" for n in self.joint_names]
        cols += [f"actual_vel_{n}" for n in self.joint_names]
        cols += [f"actual_eff_{n}" for n in self.joint_names]
        cols += [f"cmd_{n}" for n in self.joint_names]
        cols += ["left_grip_scaled", "right_grip_scaled"]
        cols += ["left_arm_published", "right_arm_published",
                 "left_limit_violation", "right_limit_violation"]
        return cols

    # ---------------------------------------------------------------- chunks
    def log_chunk(self, seq: int, steps, skip_steps: int,
                  t_req_wall: Optional[float] = None,
                  req_tick: Optional[int] = None,
                  obs_state=None,
                  rtc: Optional[dict] = None) -> None:
        """Record one full action chunk exactly as the server returned it --
        including the steps that will never be executed (truncated by the next
        chunk, or skipped as stale). The CSV only ever sees executed steps, so
        without this the discarded part of every prediction is lost.

        The keyword arguments describe the REQUEST that produced the chunk:
        `t_req_wall` (time.time() when its observation was taken), `req_tick`
        (executed-step count at that instant), `obs_state` (the joint state it
        carried) and `rtc` -- None when no seed was sent, else a dict with
        `prev_seq`, `offset`, `overlap`, `frozen`, `ramp`, `seed` (the
        (overlap, D) rows sent back), `freeze_err_mrad` and `server_ack`. All
        default to "unknown", so the v0.6.0 call signature keeps working.

        Never interrupts a run: any failure is logged and swallowed.
        """
        if self._closed:
            return
        try:
            arr = np.asarray(steps, dtype=np.float32)
            if arr.ndim != 2:
                return
            t_req = (float("nan") if t_req_wall is None
                     else float(t_req_wall) - self.t0)
            tick = -1 if req_tick is None else int(req_tick)
            state = (None if obs_state is None
                     else np.array(obs_state, dtype=np.float32).ravel())
            facts = None
            if rtc is not None:
                facts = dict(rtc)
                if facts.get("seed") is not None:
                    facts["seed"] = np.array(facts["seed"], dtype=np.float32)
            with self._chunk_lock:
                self._chunk_seq.append(int(seq))
                self._chunk_t.append(time.time() - self.t0)
                self._chunk_skip.append(int(skip_steps))
                self._chunk_data.append(arr)
                self._chunk_t_req.append(t_req)
                self._chunk_req_tick.append(tick)
                self._chunk_obs_state.append(state)
                self._chunk_rtc.append(facts)
                self._chunks_since_flush += 1
                due = self._chunks_since_flush >= CHUNK_FLUSH_EVERY
            if due:
                self._ensure_writer()
                self._chunk_wake.set()   # hand off; never write here
        except Exception as exc:  # noqa: BLE001
            if self.ros_logger is not None:
                self.ros_logger.warn(f"[data_logger] chunk log failed: {exc}")

    def _ensure_writer(self) -> None:
        if self._chunk_writer is None:
            self._chunk_writer = threading.Thread(
                target=self._writer_loop, name="chunk_writer", daemon=True)
            self._chunk_writer.start()

    def _writer_loop(self) -> None:
        """Persist the chunk store whenever asked, off the control thread."""
        while True:
            self._chunk_wake.wait()
            self._chunk_wake.clear()
            if self._chunk_stop:
                return
            try:
                self._flush_chunks()
            except Exception as exc:  # noqa: BLE001
                if self.ros_logger is not None:
                    self.ros_logger.warn(f"[data_logger] chunk flush failed: {exc}")

    def _flush_chunks(self) -> None:
        """Rewrite <run>.chunks.npz from the accumulated chunks.

        Runs on the writer thread (or inline from close(), when the run is
        already over). Snapshots the lists under the lock, then writes without
        holding it, so a slow disk never blocks the control loop.

        Uncompressed on purpose: zlib costs ~26x the time for a few percent of
        space on this data (80 ms vs 3 ms at 600 chunks), and the whole store
        is a couple of MB.

        Chunks may differ in length (a shorter checkpoint, a mid-run change),
        so they are NaN-padded to the longest seen; the reader masks NaN. The
        RTC seed rows (padded to the longest overlap sent) and the observation
        state are stored the same way. Scalars use NaN / -1 / 0 for "unknown".
        """
        with self._chunk_lock:
            self._chunks_since_flush = 0
            if not self._chunk_data:
                return
            data = list(self._chunk_data)
            seqs = list(self._chunk_seq)
            ts = list(self._chunk_t)
            skips = list(self._chunk_skip)
            t_reqs = list(self._chunk_t_req)
            req_ticks = list(self._chunk_req_tick)
            states = list(self._chunk_obs_state)
            rtcs = list(self._chunk_rtc)
        n = len(data)
        h = max(a.shape[0] for a in data)
        d = max(a.shape[1] for a in data)
        stack = np.full((n, h, d), np.nan, dtype=np.float32)
        for i, a in enumerate(data):
            stack[i, : a.shape[0], : a.shape[1]] = a

        ds = max((s.shape[0] for s in states if s is not None), default=0)
        obs_state = np.full((n, ds), np.nan, dtype=np.float32)
        max_ov = max((_int(r.get("overlap"), 0) for r in rtcs if r is not None),
                     default=0)
        rtc_seed = np.full((n, max_ov, d), np.nan, dtype=np.float32)
        rtc_applied = np.zeros(n, dtype=np.int8)
        rtc_prev_seq = np.full(n, -1, dtype=np.int32)
        rtc_offset = np.full(n, -1, dtype=np.int16)
        rtc_overlap = np.zeros(n, dtype=np.int16)
        rtc_frozen = np.zeros(n, dtype=np.int16)
        rtc_ramp = np.full(n, np.nan, dtype=np.float32)
        rtc_freeze_err = np.full(n, np.nan, dtype=np.float32)
        rtc_server_ack = np.full(n, -1, dtype=np.int8)
        for i, (s, r) in enumerate(zip(states, rtcs)):
            if s is not None:
                obs_state[i, : s.shape[0]] = s
            if r is None:
                continue
            rtc_applied[i] = 1
            rtc_prev_seq[i] = _int(r.get("prev_seq"), -1)
            rtc_offset[i] = _int(r.get("offset"), -1)
            rtc_overlap[i] = _int(r.get("overlap"), 0)
            rtc_frozen[i] = _int(r.get("frozen"), 0)
            rtc_ramp[i] = _float(r.get("ramp"))
            rtc_freeze_err[i] = _float(r.get("freeze_err_mrad"))
            rtc_server_ack[i] = _int(r.get("server_ack"), -1)
            seed = r.get("seed")
            if seed is not None and seed.ndim == 2:
                rows, cols = min(seed.shape[0], max_ov), min(seed.shape[1], d)
                rtc_seed[i, :rows, :cols] = seed[:rows, :cols]
        tmp = self.chunks_path + ".tmp.npz"
        np.savez(
            tmp,
            seq=np.asarray(seqs, dtype=np.int32),
            t_recv=np.asarray(ts, dtype=np.float64),
            skip=np.asarray(skips, dtype=np.int16),
            chunks=stack,
            # v0.7.0 -- the request behind each chunk
            t_req=np.asarray(t_reqs, dtype=np.float64),
            req_tick=np.asarray(req_ticks, dtype=np.int32),
            obs_state=obs_state,
            rtc_applied=rtc_applied,
            rtc_prev_seq=rtc_prev_seq,
            rtc_offset=rtc_offset,
            rtc_overlap=rtc_overlap,
            rtc_frozen=rtc_frozen,
            rtc_ramp=rtc_ramp,
            rtc_seed=rtc_seed,
            rtc_freeze_err_mrad=rtc_freeze_err,
            rtc_server_ack=rtc_server_ack,
        )
        # Atomic swap: a reader never sees a half-written store.
        os.replace(tmp, self.chunks_path)

    # ----------------------------------------------------------------- meta
    def write_meta(self, meta: dict) -> None:
        """Write/refresh the run's sidecar JSON.

        Called once at startup with the run's parameters, and again from
        close() to stamp the end time and row count. Safe to call repeatedly;
        each call rewrites the file from the accumulated dict.
        """
        if self._closed:
            return
        try:
            self._meta.update(meta or {})
            self._meta.setdefault("csv_file", os.path.basename(self.csv_path))
            self._meta.setdefault("started_unix", self.t0)
            self._meta.setdefault(
                "started_iso", datetime.fromtimestamp(self.t0).isoformat(timespec="seconds"))
            with open(self.meta_path, "w") as f:
                json.dump(self._meta, f, indent=2, sort_keys=True, default=str)
        except Exception as exc:  # noqa: BLE001
            # Metadata is never worth interrupting a run for.
            if self.ros_logger is not None:
                self.ros_logger.warn(f"[data_logger] could not write meta: {exc}")

    def update_meta(self, **fields) -> None:
        """Merge fields discovered mid-run (e.g. the server's first info dict,
        the observed chunk length) into the sidecar."""
        self.write_meta(fields)

    # ------------------------------------------------------------- per tick
    def log_step(self,
                 tick: int,
                 inference_seq: int,
                 horizon_idx: int,
                 actual_pos: Optional[np.ndarray],
                 actual_vel: Optional[np.ndarray],
                 actual_eff: Optional[np.ndarray],
                 cmd: np.ndarray,
                 left_grip_scaled: float,
                 right_grip_scaled: float,
                 left_arm_published: bool,
                 right_arm_published: bool,
                 left_limit_violation: bool,
                 right_limit_violation: bool,
                 latency_ms: Optional[float] = None,
                 chunk_len: Optional[int] = None,
                 skip_steps: Optional[int] = None,
                 rtc_applied: Optional[bool] = None,
                 buffer_len: Optional[int] = None) -> None:
        """Append one control-tick row: actual state vs commanded action.

        The trailing arguments default to None (-> blank cell) so any caller
        that predates them keeps working unchanged.
        """
        if self._closed:
            return
        n = len(self.joint_names)

        def vals(arr):
            """Always return exactly n values, padding with None (blank)."""
            if arr is None:
                return [None] * n
            flat = list(np.asarray(arr, dtype=float).ravel()[:n])
            if len(flat) < n:
                flat += [None] * (n - len(flat))
            return flat

        def opt_int(v) -> str:
            return "" if v is None else str(int(v))

        now = time.time()
        row = [_fmt(now), _fmt(now - self.t0), str(tick),
               str(inference_seq), str(horizon_idx), _fmt(latency_ms),
               opt_int(chunk_len), opt_int(skip_steps),
               opt_int(rtc_applied), opt_int(buffer_len)]
        row += [_fmt(v) for v in vals(actual_pos)]
        row += [_fmt(v) for v in vals(actual_vel)]
        row += [_fmt(v) for v in vals(actual_eff)]
        row += [_fmt(v) for v in vals(cmd)]
        row += [_fmt(left_grip_scaled), _fmt(right_grip_scaled)]
        row += [str(int(bool(left_arm_published))),
                str(int(bool(right_arm_published))),
                str(int(bool(left_limit_violation))),
                str(int(bool(right_limit_violation)))]

        self._f.write(",".join(row) + "\n")
        self.n_steps += 1

    # ----------------------------------------------------------------- close
    def close(self) -> None:
        if self._closed:
            return
        dur = time.time() - self.t0
        # Stop the writer, then flush inline: the run is over, so blocking is
        # free and this guarantees the last chunks reach disk.
        self._chunk_stop = True
        self._chunk_wake.set()
        if self._chunk_writer is not None:
            self._chunk_writer.join(timeout=5.0)
        try:
            self._flush_chunks()
        except Exception as exc:  # noqa: BLE001
            if self.ros_logger is not None:
                self.ros_logger.warn(f"[data_logger] final chunk flush failed: {exc}")
        # Stamp the outcome into the sidecar BEFORE flipping _closed, since
        # write_meta() is a no-op once the logger is closed.
        self.write_meta({
            "ended_unix": time.time(),
            "duration_s": round(dur, 3),
            "n_rows": self.n_steps,
        })
        self._closed = True
        try:
            self._f.flush()
            self._f.close()
        except Exception:  # noqa: BLE001
            pass
        if self.ros_logger is not None:
            self.ros_logger.info(
                f"[data_logger] wrote {self.n_steps} rows over {dur:.1f}s "
                f"-> {self.csv_path}")
