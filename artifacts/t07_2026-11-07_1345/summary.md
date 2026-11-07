# T07 — Agent 计划、验证与失败恢复（passed）

- 时间：2026-11-07 13:50 (+08:00)
- 工程：`<repo>`
- 场景：`profile_so101_physx`（SO-ARM101 + 红块 + 目标盒）

## 1. 本阶段改动

### Worker 接入

- `profiles.py`：SO-101 profile 的 `entry_module` 指向 `robotics_backends.isaaclab_physx.so101_server_entry`。
- `endpoint.py`：`SubprocessEndpoint` 不再固定 Franka worker，而是按 `SimulationSpec` 的固定
  `WorkerProfile` 解析 entry（`entry_for_profile`），profile 变化时重启 worker；仍只允许 PhysX profile。
- `so101_worker.py`：`SO101Worker` 提供
  - `observe()`：6 关节状态、嘴部 EE 位姿、`red_block`/`target_box` 对象；
  - `robot_spec()`：6 关节、单旋转夹爪、skills `observe_scene/pick/move_object/hold_pose`；
  - `run_skill()`：`pick` 调用 `SO101PickPlace`，`move_object` 为故障注入开发技能（只允许 red_block），
    `hold_pose` 用于取消测试。
- `server.py`：有 `run_skill` 的 worker 走通用分发；技能返回 `passed=false` 时发送 FAILED 事件，
  `error_code` 为 `EMPTY_GRASP/GRASP_MISSED/...`，不伪装成成功。
- `so101_server_entry.py`：SO-101 的长驻 Worker（Kit + 场景 + 回环 TCP），关闭时序沿用 T04 的有界退出。

### 过期检测（Gateway）

- `store.py`：tasks 增加 `snapshot_json`；旧库自动 `ALTER TABLE` 迁移。
- `service.py`：
  - `validate_plan()` 观察当前世界并保存对象位置快照（episode/state_version + 每个对象的 position）；
  - `execute_plan()` 在 episode/state_version 检查之外，逐个对象比较位置（容差 5 mm），任一对象移动、
    缺失或对象集合变化都返回 **409 STALE_WORLD_STATE**（retryable）。
- `submit_skill`/`move_object` 会递增 state_version，所以“计划后移动物体”一定使旧计划过期。

### 抓取变体

- `SO101PickPlace`：`GRASP_VARIANTS` 0.18 / 0.20（离线求解的 approach/grasp/lift1/lift2），
  transfer/place/retreat 共用；`grasp_x` 缺省取实测块 x，超出 ±15 mm 返回 `GRASP_POSE_UNAVAILABLE`。
- 该变体选择只覆盖两个已求解位置；任意目标仍需离线求解器生成新路点（限制，见第 5 节）。

## 2. 确定性对照流程（flow2，oracle 控制，不计作 Agent 规划）

脚本：`scripts\t07_gateway_flow.py`；证据：`flow2.json`、`flow2.log`、`gateway2.log`、
`worker_1794030172\`。`flow3` 的 worker 启动挂起（240 s 超时），作为启动 flake 记录，不用作证据。

| 步骤 | 请求 | 结果 |
|---|---|---|
| 启动 | POST /simulations（profile_so101_physx） | start job succeeded |
| 建任务/验计划 | POST /tasks、POST /tasks/{id}/plans/validate | 201 / 200 + plan_hash |
| 故障 1 | `move_object` 把块从 0.180 移到 0.184 | skill job succeeded；state_version+1 |
| 执行旧计划 | POST /tasks/{id}/execute | **409 STALE_WORLD_STATE** |
| 重观察+重规划 | GET state → 新 plan（base_state_version=新值） | validate 200 |
| 执行新计划 | execute | task job **succeeded** |
| 重置+故障 2 | reset；`move_object` 把块移到 x=0.20 | state_version 更新 |
| 首次抓取（错误变体 0.18） | plan grasp_x=0.18 → execute | task job **failed**，`GRASP_MISSED` |
| 恢复 | 重观察 → plan grasp_x=0.20 → execute | task job **succeeded** |
| 终态 | GET state | 块 `(0.232, 0.059, 0.055)`，在目标盒顶面 |

> 注：flow2 的 task/plan 由脚本生成，仅作为确定性对照；Agent 规划证据见第 3 节。
> flow2 的 JSON 里 `stale_detected=false` 是脚本读取响应嵌套的 bug；`execute_stale_plan` 记录为
> HTTP 409，响应体 `error.code=STALE_WORLD_STATE`，已修正检查逻辑。

## 3. DSH 工具链（Agent 规划，passed）

真实会话（Gateway `gateway4.db`，`profile_so101_physx`，sim `sim_2ac7680fe3c84fb692a115a6bc010361`）：

| 步骤 | 工具 | 结果 |
|---|---|---|
| 启动 | `robotics_start_simulation(profile_so101_physx, seed=7)` | job succeeded（11.4 s），skills 可见 |
| 观察/生成计划 | `robotics_inspect_scene` → `robotics_create_task` → `robotics_validate_plan` | task `task_9f3c1a7b…`；plan_hash `726a9643…` |
| 故障 1 | `robotics_run_skill(move_object, red_block → [0.184,0,0.075])` | succeeded（state_version 0→1） |
| 旧计划执行 | `robotics_execute_task(726a9643…)` | **错误 `STALE_WORLD_STATE: plan episode/state_version is stale`** |
| 重观察+重规划 | `robotics_inspect_scene` → `robotics_validate_plan`（base_state_version=1） | plan_hash `31e1a998…` |
| 执行 | `robotics_execute_task(31e1a998…)` | task job **succeeded**；块 (0.2326,0.0586,0.0550) 在盒顶 |
| 故障 2 准备 | `robotics_reset_simulation` + `move_object(red_block → [0.20,0,0.075])` | state_version 4；块 (0.2000,0,0.0750) |
| 首次抓取（错误变体） | plan `grasp_x=0.18` → validate → execute | task job **failed**，`GRASP_MISSED`；块未被扰动 |
| 重观察+重规划 | `robotics_inspect_scene` → plan `grasp_x=0.20` | valid，plan_hash `8fff87d5…` |
| 执行 | `robotics_execute_task(8fff87d5…)` | task job **succeeded**；块 (0.2322,0.0585,0.0550) |
| 关闭 | `robotics_close_simulation` | closed true |

计划中的对象（red_block）、目标区域（target_box 上方）、步骤顺序（observe → pick）和抓取变体均来自
`robotics_inspect_scene` 的观测，而不是脚本 oracle。完整时序与原始工具输出见 `dsh_tools.md`。

### reset 问题

`robotics_reset_simulation` 只换了 episode/state_version，未恢复实体初始位姿（red_block 仍在盒顶）。
已把 `SO101Worker.reset()` 改为 `sim.reset()` + `scene.reset()` + 10 步 settle；T09 的 clean restart
会重新验证该修复。本流程改用 `move_object` 设置故障 2，不依赖 reset。

## 4. 预算与恢复

- TaskSpec budget：`sim_time_s=180`、`wall_time_s=240`、`max_skill_calls=6`、`max_retries=2`。
- 每次 `pick` 的 sim 步数/时间记录在技能结果的 `sim_steps/sim_time_s`。
- 恢复日志：gateway 的 jobs/events 表保存每次 validate/execute、状态与错误码；`flow2.json`
  按时间顺序记录请求与响应。

## 5. 限制

- 抓取变体只覆盖 x=0.18/0.20；place 目标固定 (0.22, 0.05)。任意目标需要运行离线求解器并重新生成路点。
- `move_object` 是故障注入开发技能，只支持 red_block；不应出现在正常运行策略中。
- Gateway 仍是 loopback、单 worker、无认证；一次进程内 close 后需要重启 Gateway（T09 做 clean restart 演示）。
- 脚本流程不是 Agent 规划；只有第 3 节的 DSH 工具链算 T07 的 Agent 证据。

## 6. 证据文件

- `dsh_tools.md`（真实 DSH 工具链时序与结果）
- `gateway4.db`、`gateway4.log`、`worker_1794030682\`（Agent 工具链）
- `flow2.json`、`flow2.log`、`gateway2.log`、`worker_1794030172\`（确定性对照）
- `flow3.json`（worker 启动挂起的失败记录，不作为通过依据）


