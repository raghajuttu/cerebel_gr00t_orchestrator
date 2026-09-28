#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the RTC seam alignment helpers and the chunk store's RTC
arrays. Pure numpy; no ROS or ZMQ needed:

    python -m unittest tests/test_rtc.py -v       # or:  pytest tests/
"""
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from cerebel_orchestrator.gr00t_client.data_logger import InferenceLogger  # noqa: E402
from cerebel_orchestrator.gr00t_client.rtc import (  # noqa: E402
    build_seed_action, freeze_error_mrad, plan_seed, rtc_options, seed_rows)

KEYS = ("left_arm", "left_gripper", "right_arm", "right_gripper")


def chunk(H, D=16, start=0.0):
    """Synthetic (H, D) chunk whose rows are distinguishable: row i == i + start."""
    return (np.arange(H, dtype=np.float32)[:, None] + start) * np.ones((1, D), np.float32)


def load_npz(path):
    """Load and CLOSE the file, so the temp dir can be removed on Windows."""
    with np.load(path) as z:
        return {k: z[k] for k in z.files}


class PlanSeedTest(unittest.TestCase):
    def test_steady_state_prefetch_is_the_unshifted_tail(self):
        # 40-step chunk, request fired with 12 steps left in the plan: cursor 28.
        p = plan_seed(40, 28, 12, 9)
        self.assertEqual((p.horizon, p.offset, p.overlap, p.frozen), (40, 28, 12, 9))
        np.testing.assert_array_equal(p.idx, np.arange(40))
        np.testing.assert_array_equal(p.idx[-12:], np.arange(28, 40))

    def test_early_cursor_shifts_the_seed_into_the_tail(self):
        # execution_horizon 16 on a 40-step chunk: the request fires at cursor 13.
        p = plan_seed(40, 13, 12, 9)
        self.assertEqual((p.overlap, p.frozen, p.offset), (12, 9, 13))
        np.testing.assert_array_equal(p.idx[-12:], np.arange(13, 25))
        # earlier rows are real earlier steps, clamped at step 0
        self.assertEqual(p.idx[0], 0)
        self.assertTrue(np.all(np.diff(p.idx) >= 0))

    def test_overlap_is_cut_to_what_remains(self):
        p = plan_seed(40, 33, 12, 9)
        self.assertEqual((p.overlap, p.frozen), (7, 7))
        np.testing.assert_array_equal(p.idx[-7:], np.arange(33, 40))

    def test_sixteen_step_chunk_after_one_prefetch_cycle(self):
        # v0.4.0 sent prev[4:16] here while the arm was on step 9.
        p = plan_seed(16, 9, 12, 9)
        self.assertEqual((p.overlap, p.frozen), (7, 7))
        np.testing.assert_array_equal(p.idx[-7:], np.arange(9, 16))

    def test_front_padding_repeats_step_zero(self):
        p = plan_seed(16, 2, 12, 9)
        np.testing.assert_array_equal(
            p.idx, [0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13])
        np.testing.assert_array_equal(p.idx[-12:], np.arange(2, 14))

    def test_used_up_plan_gives_no_seed(self):
        self.assertIsNone(plan_seed(40, 40, 12, 9))
        self.assertIsNone(plan_seed(40, 57, 12, 9))
        self.assertIsNone(plan_seed(0, 0, 12, 9))
        self.assertIsNone(plan_seed(40, 28, 0, 0))

    def test_frozen_never_exceeds_overlap(self):
        self.assertEqual(plan_seed(40, 28, 12, 30).frozen, 12)
        self.assertEqual(plan_seed(40, 28, 12, -3).frozen, 0)


class SeedArrayTest(unittest.TestCase):
    def setUp(self):
        self.H = 40
        self.steps = chunk(self.H)
        # the server's dict form: (1, H, D) per key, canonical widths 7/1/7/1
        self.raw = {
            "left_arm": self.steps[None, :, 0:7],
            "left_gripper": self.steps[None, :, 7:8],
            "right_arm": self.steps[None, :, 8:15],
            "right_gripper": self.steps[None, :, 15:16],
        }

    def test_sent_tail_equals_canonical_seed_rows(self):
        p = plan_seed(self.H, 13, 12, 9)
        sent = build_seed_action(self.raw, p)
        for arr in sent.values():
            self.assertEqual(arr.shape[:2], (1, self.H))
            self.assertEqual(arr.dtype, np.float32)
        tail = np.concatenate([sent[k][0, -12:] for k in KEYS], axis=-1)
        np.testing.assert_array_equal(tail, seed_rows(self.steps, p))
        np.testing.assert_array_equal(tail, self.steps[13:25])

    def test_options_match_the_plan(self):
        p = plan_seed(self.H, 33, 12, 9)
        self.assertEqual(rtc_options(p, 5.0), {
            "action_horizon": 40, "rtc_overlap_steps": 7,
            "rtc_frozen_steps": 7, "rtc_ramp_rate": 5.0})

    def test_wrong_shapes_are_refused(self):
        p = plan_seed(self.H, 28, 12, 9)
        with self.assertRaises(ValueError):
            build_seed_action({"left_arm": self.steps[:, 0:7]}, p)          # no batch dim
        with self.assertRaises(ValueError):
            build_seed_action({"left_arm": self.steps[None, :16, 0:7]}, p)  # 16 != 40

    def test_seed_rows_is_a_copy(self):
        p = plan_seed(self.H, 28, 12, 9)
        s = seed_rows(self.steps, p)
        s[:] = -1
        self.assertEqual(self.steps[28, 0], 28.0)


class FreezeErrorTest(unittest.TestCase):
    def test_exact_copy_is_zero(self):
        seed = chunk(12, start=100)
        new = np.concatenate([seed[:9], chunk(31, start=500)])
        self.assertEqual(freeze_error_mrad(new, seed, 9), 0.0)

    def test_reports_largest_deviation_in_mrad(self):
        seed = chunk(12, start=100)
        new = seed.copy()
        new[3, 5] += 0.25   # 250 mrad on one joint of one frozen step
        self.assertAlmostEqual(freeze_error_mrad(new, seed, 9), 250.0, places=3)
        # the same deviation outside the frozen block is not counted
        new = seed.copy()
        new[10, 5] += 0.25
        self.assertEqual(freeze_error_mrad(new, seed, 9), 0.0)

    def test_nothing_frozen_is_nan(self):
        self.assertTrue(np.isnan(freeze_error_mrad(chunk(12), chunk(12), 0)))


class ChunkStoreRtcArraysTest(unittest.TestCase):
    def test_rtc_facts_round_trip_through_the_npz(self):
        with tempfile.TemporaryDirectory() as d:
            log = InferenceLogger(d, [f"j{i}" for i in range(16)], run_name="t")
            steps0 = chunk(40)
            log.log_chunk(0, steps0, 0, t_req_wall=log.t0 + 0.5, req_tick=0,
                          obs_state=np.zeros(16, np.float32), rtc=None)
            p = plan_seed(40, 28, 12, 9)
            seed = seed_rows(steps0, p)
            steps1 = np.concatenate([seed[:9] + 0.001, chunk(31, start=900)])
            log.log_chunk(1, steps1, 8, t_req_wall=log.t0 + 1.5, req_tick=28,
                          obs_state=np.ones(16, np.float32),
                          rtc={"prev_seq": 0, "offset": 28, "overlap": 12, "frozen": 9,
                               "ramp": 5.0, "seed": seed,
                               "freeze_err_mrad": freeze_error_mrad(steps1, seed, 9),
                               "server_ack": 1})
            log.close()
            z = load_npz(os.path.join(d, "t.chunks.npz"))
            np.testing.assert_array_equal(z["seq"], [0, 1])
            np.testing.assert_array_equal(z["skip"], [0, 8])
            np.testing.assert_array_equal(z["rtc_applied"], [0, 1])
            np.testing.assert_array_equal(z["rtc_prev_seq"], [-1, 0])
            np.testing.assert_array_equal(z["rtc_offset"], [-1, 28])
            np.testing.assert_array_equal(z["rtc_overlap"], [0, 12])
            np.testing.assert_array_equal(z["rtc_frozen"], [0, 9])
            np.testing.assert_array_equal(z["rtc_server_ack"], [-1, 1])
            np.testing.assert_array_equal(z["req_tick"], [0, 28])
            self.assertTrue(np.isnan(z["rtc_ramp"][0]))
            self.assertEqual(z["rtc_ramp"][1], 5.0)
            self.assertTrue(np.isnan(z["rtc_freeze_err_mrad"][0]))
            self.assertAlmostEqual(float(z["rtc_freeze_err_mrad"][1]), 1.0, places=2)
            self.assertEqual(z["rtc_seed"].shape, (2, 12, 16))
            self.assertTrue(np.all(np.isnan(z["rtc_seed"][0])))
            np.testing.assert_array_equal(z["rtc_seed"][1], seed)
            np.testing.assert_allclose(z["t_req"], [0.5, 1.5], atol=1e-6)
            np.testing.assert_array_equal(z["obs_state"][1], np.ones(16))
            # the pre-existing arrays are unchanged
            self.assertEqual(z["chunks"].shape, (2, 40, 16))
            np.testing.assert_array_equal(z["chunks"][1], steps1)

    def test_store_without_any_rtc_chunk_still_has_the_arrays(self):
        with tempfile.TemporaryDirectory() as d:
            log = InferenceLogger(d, [f"j{i}" for i in range(16)], run_name="t")
            log.log_chunk(0, chunk(40), 0)     # the v0.6.0 call signature
            log.close()
            z = load_npz(os.path.join(d, "t.chunks.npz"))
            self.assertEqual(z["rtc_seed"].shape, (1, 0, 16))
            np.testing.assert_array_equal(z["rtc_applied"], [0])
            self.assertTrue(np.isnan(z["t_req"][0]))
            self.assertEqual(z["req_tick"][0], -1)
            self.assertEqual(z["obs_state"].shape, (1, 0))


if __name__ == "__main__":
    unittest.main()
