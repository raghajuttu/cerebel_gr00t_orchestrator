# extras

Files the current runs do not use, kept for reference. Nothing here is
installed by `colcon build`, so none of it can be launched by name.

| File | What it was for |
|---|---|
| `missions/two_station_pick_place.yaml` | an earlier Nav2 pick-and-carry task -- its header says not to run it against the pick_scan_place checkpoint |
| `missions/nav_only.yaml` | Nav2 base test; Nav2 cannot close its loop on this chassis (no odometry, no TF) |
| `missions/policy_only.yaml` | one policy phase, operator-terminated -- superseded by `pick_then_place` |
| `launch/desk_test.launch.py` | the whole stack against fake_base / fake_arm, no robot |
| `tools/seam_report.py` | offline analysis of chunk seams from the client's per-tick CSVs |

To use one again, move it back to its folder at the repository root and rebuild.
The missions the robot runs are in `missions/`: `pick_then_place`,
`two_item_kit`, `three_station_kit`, `move_only`.
