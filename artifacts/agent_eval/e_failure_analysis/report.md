# Agent eval — e_failure_analysis

- run_id: `e_failure_analysis`
- simulation: `sim_d14e150e86a54059a5fcf38b4c8157a8` (profile_so101_physx, seed 7)
- episodes: 6 / passed 3 (pass_rate 0.50)
- wall: total 55.29 s, mean 9.21 s, median 10.04 s
- frames 58 / events 62
- failure codes: {"GRASP_MISSED": 1, "OBJECT_OUT_OF_WORKSPACE": 1, "OK": 3, "UNREACHABLE_TARGET": 1}
- failure categories: {"grasp_offset": 1, "unreachable": 2}

| # | task | status | code | passed | expected_met | wall s | frames | events |
|---|---|---|---|---|---|---|---|---|
| 1 | control_x018 | succeeded | OK | True | True | 17.61 | 17 | 15 |
| 2 | variant_x016 | succeeded | OK | True | True | 16.43 | 16 | 15 |
| 3 | place_out_of_reach | failed | UNREACHABLE_TARGET | False | True | 0.21 | 2 | 4 |
| 4 | block_too_far | failed | OBJECT_OUT_OF_WORKSPACE | False | True | 0.2 | 2 | 4 |
| 5 | stale_grasp_variant | failed | GRASP_MISSED | False | True | 3.65 | 5 | 9 |
| 6 | variant_x021 | succeeded | OK | True | True | 17.19 | 16 | 15 |

- verdict: {"ok": true, "expected_met_all": true}
