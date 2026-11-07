# E — Agent 失败分析（2026-11-07）

输入：`artifacts/agent_eval/e_failure_analysis/report.json`（6 episodes，混合失败）。

## 1. Agent 复述的失败分布（与脚本统计对照）

| 项 | Agent 读数 | `report.json` | 一致 |
|---|---|---|---|
| episodes | 6 | 6 | ✓ |
| passed | 3 | 3 | ✓ |
| pass_rate | 0.50 | 0.5 | ✓ |
| codes | OK 3 / GRASP_MISSED 1 / OBJECT_OUT_OF_WORKSPACE 1 / UNREACHABLE_TARGET 1 | 同左 | ✓ |
| categories | unreachable 2 / grasp_offset 1 | 同左 | ✓ |

## 2. 逐条失败根因

| # | task | code | 证据 | 根因 | 性质 |
|---|---|---|---|---|---|
| 3 | place_out_of_reach | UNREACHABLE_TARGET | 0.21 s、sim_step 0、块未动 | `place_x=0.30` 超出可达/放置容差（0.22±0.01，可达 0.15–0.27），**预检拒绝，无运动** | 参数/任务错误 |
| 4 | block_too_far | OBJECT_OUT_OF_WORKSPACE | 0.20 s、sim_step 30、块 (0.25, 0, 0.055) | 块被移到 x=0.25（>0.22 工作区），且从平台落到桌面（z 0.075→0.055） | 场景/任务错误 |
| 5 | stale_grasp_variant | GRASP_MISSED | 3.65 s、550 步、块 (0.1899, 0, 0.075)、5 帧 | 块在 x=0.19，却强制 `grasp_x=0.16`：夹爪在块左侧 3 cm 闭合，`grasp_centered` 失败，块未被碰动 | **参数错误**（可用默认最近变体） |

抓取失败帧（episode 5，`frame_998c9e60…`，sim_step 550）：夹爪张开、位于红块左上方，没有包住块——与“变体偏移 3 cm”一致。

## 3. 建议的下一步 run

- 只重跑 episode 5 的**场景**（块在 x=0.19）但**不传 `grasp_x`**，让 worker 选最近变体（0.19，D 阶段已验证通过）。
- 预测：`succeeded`，块进目标盒；若把该结果并入本轮统计，pass_rate 由 3/6 变为 **4/6 = 0.667**。
- 不认为需要对 episode 3/4 做“修复”：它们是任务参数/场景越界，应作为**任务校验失败**记录，而不是抓取缺陷。

## 4. 实际重跑（预测 vs 实际）

| 项 | 预测 | 实际 | 一致 |
|---|---|---|---|
| 状态 | succeeded | **succeeded** | ✓ |
| 块进目标盒 | true | (0.2287, 0.0581, 0.055)，xy 1.19 cm | ✓ |
| 合并 pass_rate | 4/6 = 0.667 | 4/6 | ✓ |
| wall | — | 16.01 s | — |

`artifacts/agent_eval/e_recheck/report.json`。没有出现“建议与结果不符”的情况。

## 5. 恢复实验（move_object → STALE_WORLD_STATE → 重规划）

全程用 DSH `robotics_*` 工具（与插件同一路径）：

1. `robotics_start_simulation`（SO-101，seed 7）→ `sim_d7701b8d…`；
2. `robotics_create_task`（`task_e0a1b2c3`）→ `robotics_inspect_scene`（state_version 0）→
   `robotics_validate_plan`（plan_hash `fce688…`）；
3. `robotics_run_skill(move_object)` 把块移动 4 mm → 块 (0.18387, 0, 0.075)，state_version 1；
4. 用旧 hash 调 `robotics_execute_task` → **`STALE_WORLD_STATE`**（无运动重放）；
5. 重新 `robotics_validate_plan`（base_state_version 1，hash `932845…`）→ `robotics_execute_task`
   → task **succeeded**（13.8 s），块到 (0.2326, 0.0586, 0.055)，在目标盒上方；
6. `robotics_close_simulation`。

记录：`recovery.json`。

## 6. 证据

- `report.json`、`episode_05.jsonl`（失败时间线：setup → submit → 550 步 → failed）
- `grasp_missed_final.png` / `grasp_missed_frame.json`（失败帧）
- `configs/agent_eval_e_recheck.json`、`artifacts/agent_eval/e_recheck/report.json`
- `recovery.json`（工具链逐步结果）
