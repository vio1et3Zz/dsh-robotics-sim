# Agent eval — baseline_pick_10

- run_id: `baseline_pick_10`
- simulation: `sim_123503c8fa544289960a1855454f914d` (profile_so101_physx, seed 7)
- episodes: 10 / passed 10 (pass_rate 1.00)
- wall: total 146.93 s, mean 14.69 s, median 14.68 s
- frames 142 / events 150
- failure codes: {"OK": 10}
- failure categories: {}

| # | task | status | code | passed | expected_met | wall s | frames | events |
|---|---|---|---|---|---|---|---|---|
| 1 | pick_018 | succeeded | OK | True | True | 14.91 | 15 | 15 |
| 2 | pick_018 | succeeded | OK | True | True | 14.55 | 14 | 15 |
| 3 | pick_018 | succeeded | OK | True | True | 14.68 | 14 | 15 |
| 4 | pick_018 | succeeded | OK | True | True | 14.46 | 14 | 15 |
| 5 | pick_018 | succeeded | OK | True | True | 14.87 | 15 | 15 |
| 6 | pick_018 | succeeded | OK | True | True | 14.6 | 14 | 15 |
| 7 | pick_018 | succeeded | OK | True | True | 14.71 | 14 | 15 |
| 8 | pick_018 | succeeded | OK | True | True | 14.62 | 14 | 15 |
| 9 | pick_018 | succeeded | OK | True | True | 14.68 | 14 | 15 |
| 10 | pick_018 | succeeded | OK | True | True | 14.85 | 14 | 15 |

- verdict: {"ok": true, "matches_t09": true, "note": "T09 used task/plan/execute; this runner calls /skills pick with the same physical scene."}
