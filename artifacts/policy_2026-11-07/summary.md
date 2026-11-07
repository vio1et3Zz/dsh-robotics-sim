# C1 vla_stub scenarios (2026-11-07)

| scenario | status | code | terminal | passed | chunks | sim_steps | elapsed s | state p95 ms | frame max ms | cancel ACK ms |
|---|---|---|---|---|---|---|---|---|---|---|
| normal | succeeded |  | succeeded | True | 100 | 1000 | 7.6 | 23.1 | 47.7 |  |
| timeout | failed | POLICY_TIMEOUT | failed |  | 82 | 820 |  | 21.3 | 47.9 |  |
| no_progress | failed | NO_PROGRESS | failed |  | 58 | 580 |  | 21.4 | 50.3 |  |
| cancel | canceled |  | canceled |  |  |  |  | 37.4 | 36.9 | 21.790699996927287 |
| ee_delta | failed | EMPTY_GRASP | failed | False | 1 | 10 | 0.16 | 63.7 | 143.2 |  |

- normal：策略完成 approach→grasp→close→lift，块抬升 39 mm（passed=true），100 chunks / 1000 steps / 7.6 s。
- timeout：fail=timeout,max_seconds=5 → POLICY_TIMEOUT，约 5 s（82 chunks/820 steps）。
- no_progress：fail=no_progress → NO_PROGRESS；reset 后的残余抖动先被计入运动，静止 15 chunks 后触发（共 58 chunks/580 steps，约 3 s）。
- cancel：t=1.5 s 取消 → canceled，ACK 21.8 ms（最终状态 canceled）。
- ee_delta：DLS 转换执行 10 步无异常；未抓取属预期（EMPTY_GRASP，1 chunk/10 steps）。
