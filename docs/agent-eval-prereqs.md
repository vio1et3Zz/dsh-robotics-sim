# Agent 驱动的仿真评估与数据闭环 — 前置工作清单

> 目标：在没有 VLA 的情况下，先把“Agent 可监控、可评测、可复盘”的仿真闭环做出来，为后续接入真实策略（BC/扩散策略/VLA）留好接口。
>
> 范围：A–E 全部可以在本机用现有 SO-101 / Franka worker 验证，不需要模型权重、不需要真机。
>
> 非目标（现在不做）：RL/IL 训练、VLA 推理接入、真机 sim2real、场景域随机化。

## 0. 现状与差距

**已有**

- Gateway + 固定 profile + 真实 Isaac Lab worker；SO-101 技能：`observe_scene`、`pick`、`hold_pose`、`move_object`。
- 16 个 DSH 工具；jobs/events/tasks/frames 的 SQLite 持久化；计划快照与 `STALE_WORLD_STATE` 检查。
- 前视 RGB/Depth 帧与元数据。

**差距（也是本清单要补的）**

| # | 差距 | 影响 |
|---|---|---|
| 1 | 技能运行期间 worker 主循环阻塞，`observe`/`capture` 排队（`server.py::run` 直接调用 `_run_skill`） | VLA/长技能运行时 Agent 看不到状态和画面 |
| 2 | Gateway 的 `get_state`/`capture_frame` 在 `async def` 里同步调 worker，worker 忙时卡事件循环 | 技能期间 `/health`、WS、GUI 都可能冻结 |
| 3 | Agent 只有 `get_job_status`，没有“读事件”工具；没有 rollout 记录和评测器 | 只能看到最终成败，无法做失败分析 |
| 4 | 抓取变体只有 x=0.18/0.20，任务矩阵太窄 | 评测样本重复，失败类型单一 |
| 5 | 没有策略接入点（连 stub 都没有） | 以后接 VLA 要临时改协议 |

## 1. 总验收标准

| 指标 | 目标 | 验证方式 |
|---|---|---|
| 技能运行中读状态 | p95 < 200 ms | 20 s `hold_pose` 期间每 0.5 s 调 `/state` |
| 技能运行中抓帧 | < 1 s，且 `sim_step` 递增 | 同上，每 2 s 抓一帧 |
| 取消 | ACK < 1 s | 技能中 `POST /jobs/{id}/cancel` |
| 事件可读 | `created/started/progress/succeeded` 齐全、分页不重不漏 | 跑一次 `pick`，读 events |
| 评测 | 固定任务跑 N 次，产出 `report.json` + 每 episode JSONL | B2 |
| 闭环 demo | 一个“失败 → 加变体 → 重跑通过”的例子 | D |

## 2. 清单（按依赖顺序）

### A. 监控地基（必须先做）

#### A1. Worker 并发 + 状态快照 + 帧缓冲

- [x] 技能在 sim 线程内**协作式**执行：每步之后回调命令循环，服务 `observe`/`capture`/`cancel`
- [x] 每 N 步发布一份 `WorldState` 快照（2 Hz；命令循环只读快照，不跨线程读张量）
- [x] 相机帧进环形缓冲（1 Hz，保留最近 8 帧，按需落盘 PNG/NPY）
- [x] `observe` 返回最新快照；`capture` 技能中返回最新缓冲帧（元数据 `buffered: true`），空闲时才 `fresh=True` 现渲
- [x] `submit` 有活动技能时返回 `WORKER_BUSY`；`reset` 在技能中同样拒绝

**改哪里**

- `python/robotics_backends/isaaclab_physx/server.py`（`_hook` 协作调度 / `_service_commands`）
- `python/robotics_backends/isaaclab_physx/worker.py`（`SnapshotStore`、`WorkerMonitor`）
- `python/robotics_backends/isaaclab_physx/so101_worker.py`（快照/帧缓冲/落盘）
- `python/robotics_backends/isaaclab_physx/so101_pick_place.py`（每步 `step_cb`）

**怎么验证**

1. 改前基线：`hold_pose(duration_s=20)` 运行中调 `/state`，会一直挂到技能结束——先把这个现象记下来。
2. 新增 `scripts/monitor_probe.py`：启动一个技能，同时按固定频率打 `/state` 和 `/frames`，记录延迟、帧 `sim_step`、取消 ACK。
3. 改后用同一脚本验收。

**通过标准**

- state p95 < 200 ms、capture < 1 s、cancel ACK < 1 s；
- 帧 `sim_step` 递增，画面随仿真变化；
- 读状态/抓帧不改变块位姿，关节值始终有限。

**约束**

- `sim.step`/render 只能在一个线程里；不要用锁让 `observe` 去抢 sim。
- 帧缓冲别无限增长；落盘只在需要时做。

> 实现说明（2026-11-07）：最初把技能放到独立线程，实测技能线程在第 ~143 步（第一次 1 Hz 相机渲染）
> 挂起——`camera.update()` 在非主线程卡死（Isaac 渲染器线程亲和），cancel 也无法 ACK；debug 日志与
> “改后”证据见 `artifacts/monitor_probe/debug/`。最终改为协作式：`step_cb` 每步回调命令循环，
> sim/render 始终在同一线程，严格满足“sim.step/render 只能在一个线程里”。

#### A2. Gateway 非阻塞

- [x] worker 绑定的调用改 `await run_in_threadpool(...)`
- [x] 技能中 `observe`/`capture` 走快照/缓冲（< 50 ms）；无缓冲帧时返回 `WORKER_BUSY`（可重试）
- [x] 技能期间 `/health` 仍然 < 100 ms，WS 事件不中断
- [x] worker 在 submit 响应前发出的事件不再丢失（按 worker job id 缓冲后回放）

**改哪里**

- `python/robotics_gateway/app.py`
- `python/robotics_gateway/service.py`
- `python/robotics_gateway/endpoint.py`

**怎么验证**

- 20 s 技能期间并发打 `/health`、`/state`、WS；观察 GUI 是否冻结。

#### A3. 事件读取 + latest frame

- [x] `store.events_for_job(job_id, after_seq, limit)`
- [x] `GET /api/v1/jobs/{job_id}/events?after_seq=&limit=`
- [x] `GET /api/v1/simulations/{id}/frames/latest`
- [x] 插件新增 `robotics_get_job_events`、`robotics_latest_frame`
- [x] 事件限流：progress 只在 10% 步发；单 job 事件上限 200 条

**改哪里**

- `python/robotics_gateway/store.py`、`app.py`
- `packages/dsh-plugin/index.js`

**怎么验证**

- 跑一次 `pick`：事件类型/数量正确，`after_seq` 分页不重不漏；
- 技能运行中取 latest frame，`sim_step` 与状态快照一致；
- 从 DSH 工具调用 `robotics_get_job_events`，返回与直接 HTTP 一致。

**A 阶段完成记录（2026-11-07）**

| 指标 | 改前 | 改后 |
|---|---|---|
| 技能中 `GET /state` | 25.9 s（阻塞到技能结束） | p95 **23.9 ms**、0 错误 |
| 技能中 `POST /frames` | 26.9 s | p50 32.8 ms、max **62.7 ms**，`sim_step` 0→315→653→982→1331 |
| 技能中 `GET /health` | 16.9 s | **15.7 ms** |
| cancel ACK | 请求被排队，返回时 job 已 succeeded | **69 ms**，65 ms 后 `canceled` |
| 事件 | 无读取接口 | `created/started/progress×4/canceled`，`next_seq` 分页 |

证据：`artifacts/monitor_probe/before/`（改前）、`after/`（20 s hold + cancel）、`pick/`（监控下真实
`pick` 成功）、`debug/`（线程方案在第一次相机渲染处挂起的日志）。

### B. 评测与记录（闭环的“数据”）

#### B1. Rollout recorder

- [x] 每个 episode 落一份 JSONL：事件、状态采样、frame_id、最终结果、耗时
- [x] 目录结构：`artifacts/agent_eval/<run_id>/episode_<n>.jsonl` + `report.json`

已实现于 `scripts/agent_eval.py` 的 `Recorder`：每行一条自描述记录（`reset`/`episode_start`/
`state`/`submit`/`event`/`frame`/`episode_end`），状态 2 Hz、事件 5 Hz 增量、帧 1 Hz。
注意帧会同时落 PNG/NPY（10 次 pick ≈ 200 MB）；只要时间线时可清理 `worker_*/frames/`。

**建议字段**

```json
{"t": 0.0, "sim_step": 0, "state_version": 0, "joints": [...], "ee": [...], "objects": {...}}
{"event_seq": 3, "type": "progress", "status": "running", "progress": 0.4, "data": {"phase": "..."}}
{"frame_id": "frame_...", "sim_step": 120, "captured_at": "..."}
```

**怎么验证**

- 跑 3 次 pick，JSONL 可逐行解析，能还原“开始 → 进度 → 成功/失败”的时间线。

#### B2. Eval runner

- [x] 任务矩阵（JSON）：`task_id / skill / parameters / seed / expected_failure`
- [x] 每次 episode 前 `reset_simulation`（不要 `close`，同一 Gateway 只允许一个 worker）
- [x] 输出指标：成功率、失败码直方图、wall/sim 时间、帧数、事件数
- [x] `report.json` 汇总 + 每个 episode 的原始 JSONL

实测（2026-11-07）：固定场景 10 次 pick **10/10**，与 T09 对齐；wall 14.46–14.91 s（mean 14.69）、
142 帧、150 事件；`--verify` 校验 report 与 JSONL 计数一致（579 行、事件序列单调）。
矩阵见 `configs/agent_eval_tasks.json`。

**建议脚本**

- `scripts/agent_eval.py`（新）

**怎么验证**

- 固定场景 10 次 pick，结果与 T09 的 10/10 对齐（有偏差要记录原因）；
- `report.json` 的计数与 JSONL 行数一致。

#### B3. 失败分类

- [x] 错误码 → 类别映射：抓空/抓偏/不可达/抓取位姿不可用/过期/忙/失联/超时/非法请求
- [x] 报告按类别给直方图
- [x] Agent 能读到同一份分类结果

映射表在 `scripts/agent_eval.py::FAILURE_CATEGORIES`，同时写进 `report.json.summary`
（`failure_codes` + `failure_categories`）。失败矩阵 `configs/agent_eval_failures.json` 实测 3/3：
块放到 x=0.16 → `GRASP_POSE_UNAVAILABLE`（0.37 s）、place_x=0.30 → `UNREACHABLE_TARGET`（0.18 s）、
成功对照 → OK；类别计数 `{grasp_pose_unavailable: 1, unreachable: 1}` 与注入一致。
`STALE_WORLD_STATE` 属计划级（`execute_plan`），不在 skill 矩阵里重复注入，已在 T07/T09 验证。

**怎么验证**

- 用 `move_object` 制造 `STALE_WORLD_STATE`；
- 把块放到 x=0.16 制造 `GRASP_POSE_UNAVAILABLE`；
- 分类计数与注入次数一致。

### C. 策略接入点（为 VLA 留的“插座”）

#### C1. `policy` / `vla_stub` 技能

- [x] 定义 `Policy` 协议：`reset()` / `act(obs) -> action chunk`
- [x] `vla_stub` 用脚本策略实现同一个协议（不加载模型）
- [x] obs：前视 RGB + 关节/末端/夹爪/对象
- [x] action：关节目标或 EE delta（EE delta 用 DLS IK 转关节目标）
- [x] 事件：`phase`（policy/execute/verify）、`step`、`progress`
- [x] 失败注入：`fail="timeout"`、`fail="no_progress"`
- [x] 取消在 action chunk 边界检查

实现：`python/robotics_backends/isaaclab_physx/stub_policy.py`（协议 + waypoint / EE-delta 两个脚本策略）；
`so101_worker.py::_policy` 负责 chunk 执行、DLS 转换、预算/停滞检查。

实测（2026-11-07，`artifacts/policy_2026-11-07/summary.md`）：

| 场景 | 结果 |
|---|---|
| 正常 lift | succeeded，块抬升 39 mm，100 chunks / 1000 steps / 7.6 s |
| `fail="timeout"`（5 s） | failed `POLICY_TIMEOUT`（82 chunks / 820 steps，约 5 s） |
| `fail="no_progress"` | failed `NO_PROGRESS`（静止 15 chunks 后触发） |
| 取消（t=1.5 s） | canceled，ACK **21.8 ms** |
| `mode="ee_delta"` | DLS 转换执行 10 步无异常（未抓取属预期 `EMPTY_GRASP`） |

**改哪里**

- `python/robotics_backends/isaaclab_physx/so101_worker.py`
- 新增 `python/robotics_backends/isaaclab_physx/stub_policy.py`

**怎么验证**

- 跑 30–60 s：state/capture 仍可用、取消 ACK < 1 s、超时/无进展能给出明确错误码；
- 与真 VLA 的接口一致（只换 `act()` 实现）。

#### C2. action/obs 日志

- [x] 每个 chunk 一条 JSONL：`step, sim_step, state_version, action, frame_id, latency_ms`
- [x] 容量上限与轮转（避免评测把磁盘写满）

实现：`so101_worker.py::_open_policy_log/_write_policy_log`，输出到 worker 目录
`policy_logs/policy_<episode>.jsonl`；字段 `chunk/sim_step/state_version/mode/targets/frame_id/
latency_ms{act,exec}/progress`；单文件 2 MB 轮转为 `policy_<ep>_1.jsonl`，单文件上限 5000 行。

**怎么验证**

- 检查字段完整、frame_id 可解析、能与 B1 的时间线对齐。

### D. 任务矩阵与场景（让闭环有东西可跑）

- [x] 用 `scripts/so101_pick_sequence_solve.py` 离线补解更多抓取变体（x=0.16/0.17/0.19/0.21）
- [x] 更新 `GRASP_VARIANTS`（6 个变体，每 1 cm），容差保持 `GRASP_VARIANT_TOLERANCE_M = 0.015`
- [x] 用 `move_object` 把块摆到不同 x，构造成功/失败分布
- [x] 固定 seed（7）与任务清单：`configs/agent_eval_tasks.json`、`agent_eval_failures.json`、
  `agent_eval_variant_fail.json`、`agent_eval_variants.json`

**怎么验证（闭环 demo）**

1. 把块放到 x=0.16 → 预期 `GRASP_POSE_UNAVAILABLE`；
2. 离线补解 x=0.16 的变体 → 重跑 → 通过；
3. 整个过程不训练任何模型，但走完了“失败 → 修复 → 验证”的闭环。

实测（2026-11-07，`artifacts/d_variants_2026-11-07/summary.md`）：

1. x=0.16 且只有 0.18/0.20 变体 → failed `GRASP_POSE_UNAVAILABLE`（0.21 s）；
2. 补解 4 个变体（位置误差 0.00 mm），0.21 额外加 z=0.175 高入口避开扫块；
3. 重跑 x=0.16/0.17/0.19/0.21 → **4/4 passed**，默认 x=0.18 回归 passed。

### E. Agent 评测工作流

- [x] Agent 跑 eval（或读 `report.json`）→ 看失败码和帧 → 分类 → 建议下一次 run
- [x] Agent 做恢复实验：`move_object` 破坏 → `STALE_WORLD_STATE` → 重规划
- [x] 记录 Agent 的结论与脚本统计的差异（这是后面“Agent 作为 failure analyst”的基线）

**怎么验证**

- Agent 复述的失败分布与 `report.json` 一致；
- Agent 的建议能对应到一次实际重跑（并说明预期变化）。

实测（2026-11-07，`artifacts/e_agent_analysis_2026-11-07/analysis.md`）：

1. 6 个 episode 的混合矩阵：3 OK / `UNREACHABLE_TARGET` / `OBJECT_OUT_OF_WORKSPACE` / `GRASP_MISSED`；
   Agent 复述的计数与类别与 `report.json` 完全一致；查看了失败帧（夹爪在块左侧 3 cm）。
2. 根因：episode 3/4 是任务参数/场景越界（预检拒绝，无运动）；episode 5 是强制 `grasp_x=0.16`
   而块在 0.19 的参数错误。
3. 建议：同场景不传 `grasp_x` 重跑 → 预测 succeeded、合并 4/6；实际重跑 **succeeded**（xy 1.19 cm）。
4. 恢复实验（DSH 工具链）：validate（hash `fce688…`）→ `move_object` 4 mm → execute
   **`STALE_WORLD_STATE`** → 用 state_version 1 重新 validate（`932845…`）→ execute **succeeded**。

差异记录：计数/类别零差异；Agent 额外给出根因标签与重跑建议（脚本报告没有这两项），这是后续
“Agent as failure analyst”的基线。

## 3. 建议顺序与验收门

| 阶段 | 内容 | 验收门 |
|---|---|---|
| 1 | A1 + A3 | 20 s 技能中 state p95 < 200 ms、capture < 1 s、取消 < 1 s、事件可读 — **已达成 2026-11-07** |
| 2 | B1 + B2 | 10 次基线评测 + `report.json` + JSONL — **已达成 2026-11-07（10/10）** |
| 3 | C1 | stub 30–60 s，可取消/超时/失败码，接口固定 — **已达成 2026-11-07** |
| 4 | D | 一个“失败 → 补变体 → 重跑通过”的闭环 — **已达成 2026-11-07（4/4）** |
| 5 | E | Agent 的失败分布与脚本统计一致 — **已达成 2026-11-07** |

## 4. 接口与命名建议（避免实现跑偏）

- Worker op：`observe`（默认返回快照）、`capture`（`fresh=false` 默认取缓冲）、`submit`（带活动技能保护）。
- Gateway：
  - `GET /api/v1/jobs/{job_id}/events?after_seq=&limit=`
  - `GET /api/v1/simulations/{id}/frames/latest`
- 插件工具：`robotics_get_job_events`、`robotics_latest_frame`。
- 错误码：新增 `WORKER_BUSY`、`NO_PROGRESS`、`POLICY_TIMEOUT`；沿用 `GRASP_MISSED`、`EMPTY_GRASP`、`UNREACHABLE_TARGET`、`GRASP_POSE_UNAVAILABLE`、`STALE_WORLD_STATE`、`CONTROL_BUSY`、`WORKER_LOST`。

## 5. 已知坑

- **mock 验证不了阻塞**：mock 的技能在另一个线程 tick，`observe` 一直能回；必须用真实 Isaac worker。
- **`reset` 不要 `close`**：`close` 后同一 Gateway 的 worker 不能复用；评测用 `reset`。
- **`state_version` 语义**：只在离散世界变化时递增；不要每控制步 bump，否则计划永远 stale。
- **事件量**：VLA/长技能容易刷爆 SQLite；限流 + 单 job 上限。
- **单 worker 串行**：10 次 pick 约 3–5 分钟（含 reset/启动开销），评测规模要提前规划。
- **GPU 8 GiB**：现在没有 VLA 没问题；接模型前先算显存预算。
- **`move_object` 是故障注入**：用于评测/扰动可以，但报告里要标注。

## 6. 最小可交付（MVP）

- A1 + A3 + B1 + B2 + C1（stub）；
- 固定场景 10 次 pick 的 `report.json` + 每 episode JSONL + 事件日志；
- 一个 stub 60 s 的运行，验证监控/取消/超时；
- 一个“失败 → 加变体 → 重跑”的闭环 demo。

粗估：A1 1–2 天，A2 半天，A3 半天，B1+B2 1 天，B3 半天，C1 1 天，D 半天到 1 天，E 1 天。
