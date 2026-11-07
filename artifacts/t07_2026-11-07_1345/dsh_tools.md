# T07 — 真实 DSH 工具链记录（2026-11-07 13:51–13:59 +08）

Gateway：`gateway4.db`，`--worker isaac`（SO-101 profile 动态选择 worker entry）。
Worker：`profile_so101_physx` → `so101_server_entry.py`；`worker_1794030682`。
Session 中实际调用（按顺序）：

1. `robotics_list_backends` → 1 个 isaaclab_physx backend（启动前 alive=false）。
2. `robotics_start_simulation(profile_id=profile_so101_physx, seed=7, idempotency_key=t07-dsh-start)`
   → `job_b924b883…`，11.4 s 后 succeeded，`sim_2ac7680fe3c84fb692a115a6bc010361`。
3. `robotics_inspect_scene` → episode `ep_d861bb04…`，state_version 0；
   red_block (0.18000, 0, 0.0750)，target_box (0.22, 0.05, 0.02)；skills
   `[hold_pose, move_object, observe_scene, pick]`。
4. `robotics_create_task`（Agent 从观察生成）：
   - task `task_9f3c1a7b2e4d4c8f9a0b1c2d3e4f5a6b`
   - goal “把红色积木放入目标盒”
   - predicate `pred_block_in_box`：red_block in region
     min (0.19,0.02,0.03) max (0.25,0.08,0.08)，tol 0.01
   - budget sim 180 s / wall 240 s / 6 calls / 2 retries
5. `robotics_validate_plan`（plan_1，base_state_version 0，`grasp_x=0.18`）
   → valid，plan_hash `726a9643…`。
6. `robotics_run_skill(skill=move_object, parameters={object_id: red_block,
   position:[0.184,0,0.075]})` → `job_037f0791…` succeeded（故障注入 1）。
7. `robotics_execute_task(plan_hash=726a9643…)` → **错误
   `STALE_WORLD_STATE: plan episode/state_version is stale`** ✓
8. `robotics_inspect_scene` → state_version 1，red_block (0.18387, 0, 0.075)。
9. `robotics_validate_plan`（plan_2，base_state_version 1，`grasp_x=0.18`）
   → valid，plan_hash `31e1a998…`。
10. `robotics_execute_task(plan_hash=31e1a998…)` → `job_62f07b85…` succeeded；
    sim_step 2090，sim_time 10.45 s；red_block (0.23257, 0.05861, 0.05500) 在目标盒顶面 ✓
11. `robotics_reset_simulation` → 新 episode `ep_bb9deb13…`，state_version 3
    （**发现 reset 只换 episode，未恢复实体初始位姿**；记录为已知问题并在 worker 中修复为
    `scene.reset()`；本流程改用 move_object 恢复场景）。
12. `robotics_run_skill(move_object → [0.20,0,0.075])` → succeeded；state_version 4。
13. `robotics_inspect_scene` → red_block (0.20000, 0, 0.07500)。
14. `robotics_validate_plan`（plan_3，base_state_version 4，**故意 `grasp_x=0.18`**）
    → valid，plan_hash `cfe2c59c…`。
15. `robotics_execute_task(plan_hash=cfe2c59c…)` → `job_629581d5…` **failed**，
    error `GRASP_MISSED` ✓（首次抓取未形成夹持；块仍在 0.20，未被扰动）
16. `robotics_inspect_scene` → state_version 5，red_block (0.20000, 0, 0.07500)，
    `pick` phase `failed`。
17. `robotics_validate_plan`（plan_4，base_state_version 5，`grasp_x=0.20`）
    → valid，plan_hash `8fff87d5…`。
18. `robotics_execute_task(plan_hash=8fff87d5…)` → `job_24a189fd…` **succeeded**；
    sim_step 2610，sim_time 13.05 s；red_block (0.23217, 0.05845, 0.05500) 在目标盒顶面 ✓
19. `robotics_close_simulation` → closed true。

## 结论

- 计划由 Agent 根据 `robotics_inspect_scene` 的观测生成（对象、目标区域、抓取变体来自观察）。
- 故障 1（validate 后移动物体）→ 旧计划 409 STALE_WORLD_STATE；重新观察并重规划后成功。
- 故障 2（首次抓取变体错误）→ task job failed GRASP_MISSED；重新观察并把 `grasp_x` 改为
  实测的 0.20 后成功。
- 两次恢复都在 task budget（180 s sim / 240 s wall、6 次技能、2 次重试）内。
- 终态块位于目标盒顶面 (0.2322, 0.0585, 0.0550)，符合 `pred_block_in_box`。

## 原始记录

- 工具调用结果已在本会话 transcript 中；Gateway DB `artifacts\t07_2026-11-07_1345\gateway4.db`
  保存 tasks/jobs/events（含 snapshot、错误码、时间戳）。
- Worker 输出：`worker_1794030682\`（stdout_1.log、stderr_1.log、worker_server_status.json）。
- Gateway 日志：`gateway4.log`。
