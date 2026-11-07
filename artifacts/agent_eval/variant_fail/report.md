# Agent eval — variant_x016_missing

- run_id: `variant_fail`
- simulation: `sim_f5f18a47780148afa894cfd303a552ff` (profile_so101_physx, seed 7)
- episodes: 1 / passed 0 (pass_rate 0.00)
- wall: total 0.21 s, mean 0.21 s, median 0.21 s
- frames 2 / events 4
- failure codes: {"GRASP_POSE_UNAVAILABLE": 1}
- failure categories: {"grasp_pose_unavailable": 1}

| # | task | status | code | passed | expected_met | wall s | frames | events |
|---|---|---|---|---|---|---|---|---|
| 1 | pick_x016_missing_variant | failed | GRASP_POSE_UNAVAILABLE | False | True | 0.21 | 2 | 4 |

- verdict: {"ok": true, "expected_met_all": true}
