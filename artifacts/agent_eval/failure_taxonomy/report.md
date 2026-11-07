# Agent eval — failure_taxonomy

- run_id: `failure_taxonomy`
- simulation: `sim_2a34754d193346beaeaab5a418fc4077` (profile_so101_physx, seed 7)
- episodes: 3 / passed 1 (pass_rate 0.33)
- wall: total 15.17 s, mean 5.06 s, median 0.37 s
- frames 18 / events 23
- failure codes: {"GRASP_POSE_UNAVAILABLE": 1, "OK": 1, "UNREACHABLE_TARGET": 1}
- failure categories: {"grasp_pose_unavailable": 1, "unreachable": 1}

| # | task | status | code | passed | expected_met | wall s | frames | events |
|---|---|---|---|---|---|---|---|---|
| 1 | grasp_pose_unavailable | failed | GRASP_POSE_UNAVAILABLE | False | True | 0.37 | 2 | 4 |
| 2 | unreachable_place | failed | UNREACHABLE_TARGET | False | True | 0.18 | 2 | 4 |
| 3 | baseline_control | succeeded | OK | True | True | 14.62 | 14 | 15 |

- verdict: {"ok": true, "expected_met_all": true}
