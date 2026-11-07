# DSH Robotics 执行进度

- 工程：`<repo>`
- 计划：`..\DSH_EXECUTION_PLAN.md`
- 最后更新：2026-11-07T20:10:00+08:00

> **状态**：**T00–T09 全部通过**。T09 固定验收 10/10 成功（修正 reset 缺陷后；首次 1/10
> 失败记录保留），两类故障恢复 2/2，取消 ACK 0.217 s，Worker 失联 `WORKER_LOST`，
> clean restart 成功；GPU 峰值 5147 MiB、关闭后 1422 MiB。可选 OP01/OP02 未开始。
>
> **增量**：A1/A2/A3（Agent 监控地基）已完成——20 s 技能中 state p95 23.9 ms、帧 62.7 ms、
> cancel ACK 69 ms；`robotics_get_job_events`、`robotics_latest_frame` 已热重载上线（16 个工具）。
> B1/B2/B3（评测与记录）已完成——固定场景 10 次 pick **10/10**、失败矩阵 3/3、JSONL 与 report 计数一致。
> C1/C2（策略接入点）已完成——`vla_stub` 正常 lift 通过，timeout/no-progress 有明确错误码，
> 取消 ACK 21.8 ms，EE-delta DLS 与 action 日志可用。
> D（抓取变体闭环）已完成——补解 x=0.16/0.17/0.19/0.21，x=0.16 从 `GRASP_POSE_UNAVAILABLE`
> 到 4/4 通过。
> E（Agent 失败分析）已完成——分布复述与 report 一致，建议→重跑成功，`STALE_WORLD_STATE` 恢复成功。

## 状态总览

| 任务 | 状态 | 证据 |
|---|---|---|
| T00 固定基线与宿主接口 | passed | `artifacts\t00_2026-11-07_0015\baseline_manifest.json`、`doctor.json`（exit 0） |
| T01 协议与适配器骨架 | passed | `artifacts\t01_2026-11-07_0045\summary.json`；Python 17 tests OK；Node 30/30 |
| T02 真实 PhysX Worker | passed | `artifacts\t02_2026-11-07_0150\summary.json`；11 项检查全 true；标定误差 2.57e-05 rad |
| T03 Gateway 与可确认取消 | passed | `artifacts\t03_2026-11-07_0210\summary.json`；真实 cancel 0.124 s；service 7/7、HTTP 3/3 |
| T04 DSH 插件与首次演示 | passed | `artifacts\t04_2026-11-07_1150\summary.md`；真实工具调用；cancel ACK 46 ms |
| T05 SO-ARM101 资产与标定 | passed | `artifacts\t05_2026-11-07_1120\summary.md`；6 关节 + 旋转夹爪真实标定；物体场景通过 |
| T06 抓取/放置技能与物理验证 | passed | `artifacts\t06_2026-11-07_1130\summary.md`；3/3 一致 pick/place；抬升 43 mm；三类明确失败 |
| T07 Agent 计划与失败恢复 | passed | `artifacts\t07_2026-11-07_1345\summary.md`；DSH 工具链 stale→恢复、GRASP_MISSED→恢复 |
| T08 相机帧接口与 UI | passed | `artifacts\t08_2026-11-07_1410\summary.md`；真实 RGB/Depth、元数据、帧随状态变化、清理实测 |
| T09 固定工程验收 | passed | `artifacts\t09_2026-11-07_1435\summary.md`；10/10、2/2 恢复、取消 0.217 s、WORKER_LOST、clean restart |
| A1–A3 Agent 监控地基 | passed | `artifacts\monitor_probe\after\summary.md`、`pick\summary.json`、`reuse\summary.json`、`plugin_reload.json` |
| B1–B3 Agent 评测与记录 | passed | `artifacts\agent_eval\baseline_pick_10\report.json`（10/10）、`failure_taxonomy\report.json`（3/3） |
| C1–C2 策略接入点 | passed | `artifacts\policy_2026-11-07\summary.json`（5 场景、policy 日志、取消 21.8 ms） |
| D 抓取变体闭环 | passed | `artifacts\d_variants_2026-11-07\summary.md`；变体 4/4、失败→修复→重跑 |
| E Agent 失败分析 | passed | `artifacts\e_agent_analysis_2026-11-07\analysis.md`（分布一致、建议→重跑成功）、`recovery.json` |
| OP01 Newton 兼容 | pending | — |
| OP02 Python Server 接入 | pending | — |

## T00 — 固定软件基线与宿主接口（passed）

### 修改

新建（未改动 Isaac 上游源码、profile 或已有实验）：

- `docs\compatibility.md`
- `configs\local.example.json`
- `scripts\doctor.ps1`
- `scripts\isaac_metadata.py`
- `scripts\read_asar_package.py`
- `docs\execution\PROGRESS.md`
- `docs\execution\task_state.json`
- `artifacts\t00_2026-11-07_0015\`：`baseline_manifest.json`、`doctor.json`、`dsh_package.json`、`file_hashes.json`、`isaaclab_commit.txt`、`isaaclab_describe.txt`、`isaaclab_status.txt`、`isaaclab_worktree.diff`、`python_packages.txt`

### 运行命令与退出码

| 命令 | 退出码 | 说明 |
|---|---|---|
| `git -C <isaac-root>\IsaacLab describe --tags --always --dirty` | 0 | `v3.0.0-beta2.patch1-dirty` |
| `git -C <isaac-root>\IsaacLab rev-parse HEAD` | 0 | `ffff603eafc6b74264a5261cc0183d6a65390d78` |
| `git -C <isaac-root>\IsaacLab status --porcelain=v1` | 0 | 1 个修改文件 + 1 个未跟踪 `_` |
| `git -C <isaac-root>\IsaacLab diff` | 0 | 12 行本地补丁，已保存 |
| `<isaac-root>\env_isaaclab\Scripts\python.exe -m pip freeze` | 0 | 保存到 `python_packages.txt` |
| `powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts\doctor.ps1` | 0 | baseline OK，18 项检查全部 ok |
| `powershell.exe ... -File scripts\doctor.ps1 -Json` | 0 | 写入 `doctor.json` |

注：直接 `& script.ps1` 会被 `RemoteSigned` 策略拒绝，必须用 `powershell.exe -ExecutionPolicy Bypass -File` 启动；已写入 `docs\compatibility.md` 第 5.2 节。

### 证据

- `artifacts\t00_2026-11-07_0015\baseline_manifest.json`：版本、Git 提交、文件哈希、GPU、命令与退出码、限制。
- `artifacts\t00_2026-11-07_0015\doctor.json`：三段式检查（installed / services / verified）。
- `artifacts\t00_2026-11-07_0015\isaaclab_worktree.diff`：本地补丁快照（TorchScript 非 ASCII 导出路径修复）。
- `docs\compatibility.md`：宿主工具/Job 契约、版本解释、本机约束。

### 验收对照

| 验收项 | 结果 |
|---|---|
| doctor 区分“包已安装 / 服务存活 / 任务已验证” | 通过；`installed` 18 项 ok，`services.gateway=not_running`（T03 前预期），`verified.tasks` 为空 |
| 解释 Lab 产品版本与包版本不同 | 通过；`compatibility.md` 第 2 节 |
| DSH 工具注册 API 有本机来源 | 通过；live Inspect + `dsh-tool-todo\lib\index.js` |
| 不依赖旧盘符路径 | 通过；`paths.legacyDrives` 为 ok |

### 剩余问题

- Gateway/Worker 尚不存在（T02/T03）；`not_running` 是预期状态。
- `ctx.jobs.start` 的 controller 与 kind 约束未实测（T04）。
- SO-ARM101 资产未在本地缓存，需 T05 实际下载并核对许可。
- 中文工作区可写普通文件；TorchScript/ONNX 导出必须走 ASCII 目录。

### 下一动作

T01：定义 `RuntimeManifest`、`BackendCapabilities`、`RobotSpec`、`SimulationSpec`、`WorldState`、`SkillInvocation`、`TaskSpec`、`TaskPlan`、`JobEvent`、`ErrorInfo`，Pydantic 单一源导出 JSON Schema 与 TypeScript 类型，并实现 fake backend 与共享样例校验。

## T01 — 协议与适配器骨架（passed）

### 修改

- `python\robotics_protocol\`：`base.py`、`manifest.py`、`capabilities.py`、`robot.py`、`simulation.py`、`state.py`、`skills.py`、`tasks.py`、`jobs.py`、`errors.py`、`export_schema.py`、`__init__.py`。
- `python\robotics_backends\`：`base.py`（`SimulationBackend` 抽象）、`fake.py`、`profiles.py`（固定 profile 白名单）、`__init__.py`。
- `schemas\v0.1\protocol.bundle.json`、`models.json`（由 Pydantic 生成）。
- `packages\robotics-protocol\`：`types.ts`（生成）、`validate.mjs`（Ajv + 语义镜像）、`tsconfig.json`、`package.json`。
- `tests\fixtures\protocol\`：`build_cases.py`、`cases.json`（30 例：16 合法 / 14 非法）。
- `tests\unit\`：`test_protocol.py`、`test_fake_backend.py`。
- `scripts\export_protocol.py`、`scripts\export_protocol.ps1`。

### 运行命令与退出码

| 命令 | 退出码 | 结果 |
|---|---|---|
| `python scripts\export_protocol.py` | 0 | 生成 bundle 与 `types.ts` |
| `python -m unittest discover -s tests/unit -v` | 0 | 17 tests OK |
| `node validate.mjs`（`packages\robotics-protocol`） | 0 | 30 cases，0 failures |
| `tsc -p tsconfig.json`（`packages\robotics-protocol`） | 0 | 生成类型通过 strict 编译 |

### 证据

- `artifacts\t01_2026-11-07_0045\summary.json`：命令、退出码、样例计数、生成文件哈希、限制。
- 同目录：`export_protocol.txt`、`python_unittest.txt`、`node_validate.txt`、`tsc.txt`。

### 验收对照

| 验收项 | 结果 |
|---|---|
| TS/Python 校验同一套合法与非法样例 | 通过；30 例双端一致（非法样例含单位、数组长度、无穷值、DAG、能力状态） |
| 单位 / 数组长度 / 无穷值被拒绝 | 通过；`robot_joint_unit_mismatch`、`world_state_joint_length`、`skill_invocation_nonfinite_timeout` |
| 未知技能 / 不支持组合被拒绝 | 通过；fake backend 的 `UNSUPPORTED_CAPABILITY` 与 profile/robot 不匹配测试 |
| mock 不得判为真实后端通过 | 通过；`mock verified` 被 Pydantic/Ajv 拒绝，fake 能力记录 `verified=False` |

### 剩余问题

- `validate.mjs` 手工镜像了 Pydantic 的模型级校验；以后新增 `model_validator` 必须同步更新，否则双端会漂移。
- `plan_hash` 的等值校验只在 Python/Gateway 侧执行；Ajv 不校验哈希内容。
- `packages\robotics-protocol\node_modules` 与 `.npm-cache` 是本地工具链，不属于交付源码。
- fake backend 只覆盖协议行为，**不能**作为仿真验收。

### 下一动作

T02：用真实 SDK 启动 Franka PhysX Worker，完成 start→ready→状态快照→有界关节动作→reset→close，并保存真实物理响应证据。

## T02 — 真实 Franka PhysX Worker（passed）

### 修改

- `python\robotics_backends\isaaclab_physx\`：`scene.py`（本地 Franka 场景 + 程序化地面）、`worker.py`（生命周期、状态快照、标定）、`entry.py`（Kit 启动、清单、拒绝检查、日志）、`__init__.py`。
- `scripts\smoke_worker.ps1`：真实启动与 stdout/stderr 归档。
- `assets\isaac6\FrankaPanda\`：从 Isaac 6.0 S3 镜像的 Franka 资产树（39 MB，含 LICENSE）。
- `docs\compatibility.md` 第 6 节：资产 404 与本地镜像策略。
- `artifacts\t02_2026-11-07_0150\`：最终通过证据；`0050`/`0100`/`0110` 保留为失败证据。

### 运行命令与退出码

| 命令 | 退出码 | 结果 |
|---|---|---|
| `powershell -ExecutionPolicy Bypass -File scripts\smoke_worker.ps1 -OutputDir artifacts\t02_2026-11-07_0150` | 0 | 11 项检查全 true |
| 同脚本（`0050`/`0100`/`0110`） | 1 | 远程资产不可用，作为失败证据保留 |

### 证据

- `artifacts\t02_2026-11-07_0150\summary.json` / `summary.md`
- `result.json`：检查项、before/after/reset 状态、标定测量、耗时
- `runtime_manifest.json`：版本、Git、GPU、资产哈希
- `stdout.log` / `stderr.log` / `worker.log`

### 验收对照

| 验收项 | 结果 |
|---|---|
| 真实状态数组、命令目标与物理响应可核对 | 通过；9 关节、目标 0.049921、实测 0.049896、误差 2.57e-05 rad |
| 状态与坐标无非有限值 | 通过；`state_finite=true` |
| 非法动作在写入前被拒绝 | 通过；超限 delta 与未知关节均未改变 `sim_step`/`state_version` |
| 记录 runtime manifest | 通过；含 `franka.usd` SHA-256 与 Git 提交 |
| 关闭后资源释放 | 通过；`close()` 幂等且进程正常退出 |
| 现有 box-physics 9 环境回归 | 未运行（本阶段未将其作为回归；其 ≥9 环境约束保持不变） |

### 剩余问题

- Worker 仍是同步调用，没有命令队列、取消、网络与 Job（T03）。
- `body_quat_w` 的 xyzw 顺序沿用 `root_quat_w` 的核查结论，尚未独立验证。
- 远程 Nucleus 资产在本机不可用；T05 的 SO-ARM101 必须本地镜像并核对许可。
- `--headless` 已弃用，后续脚本可改用 `--viz none`。

### 下一动作

T03：实现 Gateway、持久 Job、幂等键、单实例控制权与“Worker 停止确认后才 CANCELED”的取消语义。

## T03 — Gateway、Job 与可确认取消（passed）

### 修改

- `python\robotics_gateway\`：`store.py`（SQLite）、`service.py`（Job 状态机/幂等/取消/plan 顺序执行）、`endpoint.py`（Loopback + Subprocess 两种 WorkerEndpoint）、`app.py`（REST/WS）、`__main__.py`、`__init__.py`。
- `python\robotics_backends\isaaclab_physx\`：`server.py`（TCP JSON-lines Worker）、`server_entry.py`（长驻 Worker 入口）；`worker.py` 增加取消检查与 `SkillCanceled`。
- `scripts\start_gateway.ps1`、`scripts\t03_real_smoke.py`。
- `tests\unit\test_gateway_service.py`、`tests\integration\test_gateway_http.py`。

### 运行命令与退出码

| 命令 | 退出码 | 结果 |
|---|---|---|
| `python -m robotics_gateway --port 8714 --db ... --worker isaac` | 运行中 | Gateway + 真实 Worker |
| `python scripts\t03_real_smoke.py --base http://127.0.0.1:8714` | 0 | start→skill→cancel→close 全通过 |
| `python -m unittest tests.unit.test_gateway_service -v` | 0 | 7 tests OK |
| `python -m unittest tests.integration.test_gateway_http -v` | 0 | 3 tests OK |

### 证据

- `artifacts\t03_2026-11-07_0210\summary.json` / `summary.md`
- `service_tests.txt`、`http_tests.txt`
- `worker_1793984681\stdout.log`、`stderr.log`
- `gateway.db`（jobs/events/idempotency/tasks）

### 验收对照

| 验收项 | 结果 |
|---|---|
| 重复 execute / 幂等 | 通过；相同幂等键返回同一 Job，只产生一个 skill Job |
| 并发 execute/reset | 通过；live Job 时 reset/close 返回 409 CONTROL_BUSY |
| 取消过程中重复取消 | 通过；第二次 cancel 返回 202 且状态 terminal |
| 通信断开 / Worker 退出 | 通过；poller 检测 `alive=false` 并把 live Job 标为 `WORKER_LOST`，有单测 |
| Gateway 重启 | 通过；`mark_live_jobs_lost()` 把 pending/running/canceling 标为 failed，不重放动作 |
| 过期 episode | 通过；`STALE_WORLD_STATE` 409 |
| 事件重连 | 通过；WebSocket `after_seq` 补取 |
| 真实动作取消 | 通过；hold 1.5 s 的标定在 0.124 s 内停止，`state_version` 0→1 |

### 剩余问题

- Gateway 复用 Isaac Python 环境，尚未拆独立 venv；只导入 fastapi/uvicorn，不导入 Kit/Torch。
- 无认证/TLS；仅 loopback。
- plan 执行器仅顺序提交，预算与重规划在 T07 完善。
- `worker_server_status.json` 在 Kit 关闭时序下偶尔缺失；以 stdout/stderr 与 Job 记录为准。

### 下一动作

T04：把 Gateway 封装为 DSH Host 插件工具（`robotics_*`），完成真实 DSH 会话中的启动、查询、执行、取消、关闭演示。

## T04 — DSH 插件与首次真实接口演示（passed）

### 最终真实演示（2026-11-07 11:10）

- 工具：`robotics_list_backends` → `robotics_start_simulation(profile_dev_franka_physx)` → `robotics_inspect_scene` → `robotics_run_skill(hold_pose, 20s)` → `robotics_cancel_job` → `robotics_get_job_status` → `robotics_inspect_scene` → `robotics_close_simulation`。
- 启动：Job `job_f9a2f4df...`，6.51 s 成功，simulation `sim_5d52ca7c...`。
- 取消：Job `job_76e677a2...` 在 progress 0.40975 时取消；`canceling` → `canceled` 的 Worker ACK 为 **46 ms**。
- 取消后状态：`state_version` 0→1，`sim_time_s=8.58`，技能状态 `canceled / "cancel acknowledged"`。
- 关闭：`closed=true`；Worker 写 `worker_server_status.json`（`status=closed`）并退出。
- 修复：shim 进程存活误判、Worker socket 10 s 超时、Kit 关闭挂起（有界 `os._exit`）、短技能来不及取消（新增 `hold_pose`）。

### 已完成

- `packages\dsh-plugin\`：`package.json`、`cordis.patch.yml`、`index.js`、`locale\en.json`、`locale\zh.json`、`icon.svg`。
- 12 个工具：`robotics_list_backends`、`start_simulation`、`inspect_scene`、`get_robot_state`、`run_skill`、`get_job_status`、`cancel_job`、`reset_simulation`、`close_simulation`、`create_task`、`validate_plan`、`execute_task`。
- `node --check index.js` 通过。
- 插件已安装并生效（profile `package.json` 含 `@local/dsh-robotics`；`Tool.listTools` 返回全部工具）。
- `Config.listConfigs` 的 `status=absent` 是因为插件未导出 `Config` schema，不代表未加载。

### 证据

- `artifacts\t04_2026-11-07_1150\summary.md`
- `artifacts\t04_2026-11-07_1150\gateway_run.json`（2 Job、205 事件、1 simulation）
- `artifacts\t04_2026-11-07_1150\worker_1794020999\`（stdout/stderr、worker_server_status.json）
- 本会话真实工具调用记录（见 summary.md 的时序表）

### 剩余问题

- `robotics_run_skill` 是开发接口；正式任务用 task/plan 工具，T06/T07 完善。
- Gateway 仍为 loopback、无认证；独立 venv 未拆。

## T05 — SO-ARM101 资产、标定与场景（passed）

### 已完成

- 从 Isaac 6.0 S3 镜像 `RobotStudio/so101_new_calib`（6 文件、23,256,986 B、Apache-2.0），主 USD SHA-256 `04ef3043…89efb3`。
- `so101_scene.py` 加载本地 USD；缺失时抛 `ASSET_NOT_READY`，不会替换成 Franka。
- 6 个可动关节（5 臂 + 1 旋转夹爪）限位/方向/索引标定；夹爪命令 0→1 rad，实测 0→0.9992。
- 静止保持漂移 0.0349 rad；物体场景红块落稳在 z=0.015，目标盒存在。
- 交付 `robot_spec.json`、`joint_mapping.json`、`asset_manifest.json`、`scene.json`。

### 运行命令与退出码

| 命令 | 退出码 | 结果 |
|---|---|---|
| `python ... so101_entry.py --headless --output_dir artifacts\t05_...\calib` | 0 | 7 项标定检查全 true |
| `python ... so101_objects_entry.py --headless --output_dir artifacts\t05_...\objects4` | 0 | 4 项物体/接触检查全 true |
| `python scripts\build_so101_specs.py ...` | 0 | 4 个 JSON 交付物 |

### 证据

- `artifacts\t05_2026-11-07_1120\summary.md`
- `artifacts\t05_2026-11-07_1120\calib\so101_calibration.json`
- `artifacts\t05_2026-11-07_1120\objects4\so101_objects.json`
- `artifacts\t05_2026-11-07_1120\so101_usd_inspection.json`
- `embodiments\so_arm101\*`

### 验收对照

| 验收项 | 结果 |
|---|---|
| 所有逻辑关节唯一映射 | 通过；6 个 logical_name 与索引唯一 |
| 实际运动方向/夹爪开闭可观察 | 通过；+0.05 rad 命令均得正位移；夹爪 0.9996 rad 行程 |
| 目标与测量状态符合限位 | 通过；全部有限且落在 USD 限位内 |
| 场景物体尺寸/接触有效 | 通过；红块落稳 z=0.015，未穿透桌面 |

### 剩余问题

- 夹爪 `max_width_m=0.04` 为估计，T06 用真实抓取标定。
- 执行器增益为项目自有值，非 USD drive 原值。
- 目标盒当前为运动学静态体，T06 放置验证需物理容差。

### 下一动作

T06：实现 SO-101 的 observe/home/move_joint/gripper/pick/place 与独立物理验证。

## T06 — 抓取/放置技能与物理验证（passed）

### 关键发现

- 夹爪不是“palm 原点夹持”：碰撞网格显示两个上下夹条，开口中心在 palm 局部
  `(0.0071, 0, -0.085)`，固定夹条内表面 `x=-0.0079`、活动夹条内表面 `x=+0.0099`，铰链 `(0.0202, 0.0188, -0.0234)`。
- 用 USD rest 变换建立 5 轴运动学链（rest 误差 1.54e-05 m），离线求解顶抓路点；
  顶抓在 x=0.16–0.26 精确可达，侧抓 +X 物理 IK 收敛差，最终采用顶抓。
- 抓取 y 偏移 −5 mm（否则固定夹条与块面间隙为 0，下降会推块）；
  下降/闭合时保持上一路点指令，不再把臂目标重置为实测位姿。

### 真实结果

- 三次独立启动的完整 pick/place 结果逐位一致；块被夹起 **43 mm**、转移到 `(0.226, 0.056)`
  目标盒上方并释放，最终 `(0.230, 0.058, 0.055)` 停在盒顶。
- 阶段谓词：`grasp_centered`、`lifted`、`block_follows_mouth`、`carried_to_place`、
  `placed_in_target_region`、`released` 全 true。
- 技能类 `python\robotics_backends\isaaclab_physx\so101_pick_place.py`：
  预检（可达区域/物体存在）、阶段测量、结构化失败、取消保持位姿。

### 明确失败（真实运行）

| 注入 | 结果 |
|---|---|
| 抓取校验后移走物块 | `EMPTY_GRASP`，`lifted=false` |
| 物块移到 x=0.20 仍用固定轨迹 | `GRASP_MISSED`，`grasp_centered=false` |
| place=(0.30,0.20) | `UNREACHABLE_TARGET`，关节未动作 |

### 证据

- `artifacts\t06_2026-11-07_1130\summary.md`（完整数据、几何、路点表、限制）
- `waypoint_pick3/4/5.json`（三次一致）、`skill_success3.json`
- `skill_empty.json`、`skill_grasp_missed.json`、`skill_unreachable.json`
- `gripper_points.json`、`gripper_views.png`、`descent_diag1.json`
- `pick_waypoints2.json`、`kinematics_sweep2.json`

### 剩余问题

- 固定路点只覆盖该场景；任意目标需离线求解器生成新路点（T07 重规划会用到）。
- 目标盒是 kinematic 静态体；成功以块最终位姿判定。
- Kit 启动偶发无输出退出，重跑成功；已记录，不作为通过依据。

### 下一动作

T07：SO-101 Worker 接入 Gateway，Agent 生成 TaskSpec/typed DAG，加入对象位姿过期检测和
两类故障恢复（计划后移动物体、首次抓取失败后重规划或停止）。

## T07 — Agent 计划、验证与失败恢复（passed）

### 已完成

- `SubprocessEndpoint` 按固定 `WorkerProfile` 选择 worker entry（SO-101 → `so101_server_entry.py`），
  profile 变化时重启 worker；仍只允许 PhysX profile。
- `SO101Worker`：6 关节 + 嘴部 EE + `red_block`/`target_box` 的 `WorldState`；
  skills `observe_scene/pick/move_object/hold_pose`；`pick` 调 `SO101PickPlace`，
  `move_object` 是故障注入开发技能。
- `WorkerServer`：有 `run_skill` 的 worker 走通用分发；技能 `passed=false` → Job FAILED +
  `error_code`（`GRASP_MISSED` 等），不伪装成功。
- `SO101PickPlace` 增加抓取变体 0.18/0.20；`grasp_x` 缺省取实测块 x。
- Gateway：`validate_plan()` 保存 world snapshot（episode/state_version/对象位置）；
  `execute_plan()` 在原有检查外比较对象位置（5 mm 容差），变化/缺失/集合变化 → 409 `STALE_WORLD_STATE`。

### 真实 DSH 工具链（2026-11-07 13:51–13:59）

- 启动 `profile_so101_physx`（`sim_2ac7680f…`，worker_1794030682）。
- Agent 用 `robotics_inspect_scene` 生成 task/plan（red_block、目标区域、步骤来自观测）。
- 故障 1：validate 后 `move_object` 把块移 4 mm → `robotics_execute_task` 报
  `STALE_WORLD_STATE`；重观察、新 plan（base_state_version=1）→ task succeeded（块到盒顶）。
- 故障 2：块移到 x=0.20，首次 plan `grasp_x=0.18` → task failed `GRASP_MISSED`；
  重观察、plan `grasp_x=0.20` → task succeeded（块到盒顶 (0.2322,0.0585,0.0550)）。
- `robotics_reset_simulation` 发现只换 episode、不恢复实体；worker 已改为
  `sim.reset()+scene.reset()+settle`，T09 复验。

### 证据

- `artifacts\t07_2026-11-07_1345\summary.md`、`dsh_tools.md`
- `gateway4.db`/`gateway4.log`、`worker_1794030682\`、`flow2.json`（确定性对照）

### 剩余问题

- 抓取变体只覆盖 x=0.18/0.20；place 固定 (0.22,0.05)；任意目标需离线求解器重算路点。
- `move_object` 是故障注入技能，只支持 red_block。
- Gateway 单 worker、loopback、无认证；close 后需重启 Gateway（T09 做 clean restart）。
- Kit 启动偶发挂起（flow3 首次 240 s 超时），重试成功。

### 下一动作

T08：前视相机 RGB/Depth 帧、帧元数据、`GET /api/v1/frames/{id}` 与最小 UI 信息。

## T08 — 相机帧接口与最小 UI（passed）

### 已完成

- `SO101CameraSceneCfg`：前视 pinhole 相机（640×480，rgb + distance_to_image_plane，
  world pose (0.38,-0.42,0.30)）+ dome light；`build_so101_object_scene(with_camera=True)`。
- `SO101Worker.capture_frame()`：真实 RTX 渲染，保存 PNG/NPY，返回完整元数据
  （frame/camera、episode、state_version、sim_step/time、时间戳、尺寸、格式、深度单位/类型、
  fx/fy/cx/cy、world 外参、rgb/depth 范围、无效深度比例、capture_ms）。
- Gateway：`POST /simulations/{id}/frames`、`GET /frames/{id}`、`/rgb.png`、`/depth.npy`、
  `GET /ui`（health/job/帧最小页面）。
- 插件：`robotics_capture_frame`、`robotics_get_frame`（源码 + node check；当次记录时 Host 尚未重载）。

### 真实结果

- frame1（state_version 0）→ `move_object` 到 x=0.20 → frame2（state_version 1，sim_step 30）；
  两帧 SHA256 不同，画面中物块右移；RGB 640×480 uint8，深度 480×640 float32、0.4006–1.7674 m、
  8.47% 无效；内参 fx=fy=733.0，cx=320，cy=240。
- 相机 worker 峰值 GPU 5022 MiB；close 后 1430 MiB，`worker_server_status=closed`，进程退出；
  帧在 close 后仍通过 Gateway 可读。

### 证据

- `artifacts\t08_2026-11-07_1410\summary.md`
- `frame1_meta.json`/`frame1_rgb.png`/`frame1_depth.npy`、`frame2_*`
- `camera_probe5.json`（全权限预编译）、`camera_probe6.json`（沙箱读缓存）
- `gateway5.db`、`gateway5.log`、`worker_1794032636\`

### 剩余问题

- Warp 相机 kernel 需预编译 ASCII 缓存（`DSH_ROBOTICS_WARP_CACHE`，默认
  `%USERPROFILE%\AppData\Local\Temp\dsh_robotics_warp`）；中文工作区路径下直接编译会失败。
- 只有前视相机，无腕部相机；按需单帧，无视频流。
- 帧是真实场景渲染，控制仍用 ground truth；未实现视觉闭环抓取。
- 新加的两个 DSH 工具在插件热重载后注册；已用 `plugin_manager` 验证（见文末运维记录），
  无需重启 Host 进程。

### 下一动作

T09：固定 eval manifest、10 次 pick/place（≥8 成功）、故障恢复/取消/失联/clean restart、
资源与时序报告、README 与可安装交付。

## T09 — 固定工程验收与交付（passed）

### 验收结果

- Manifest：`docs\execution\eval_manifest.json`（运行前固定场景/种子/预算/成功条件）。
- 固定场景 10 次：**10/10 成功**（`eval2.json`，pass_rate=1.0，wall 14.9–15.9 s，
  sim 10.3–13.05 s），块最终在目标盒顶面。
- 第一次 eval 只 1/10：reset 未恢复物体初始位姿（`OBJECT_OUT_OF_WORKSPACE`），
  失败记录保留；定位 `RigidObject.reset` 只清外力，worker 改为显式恢复块位姿/关节
  （`reset_probe.json`，块误差 <1 mm），复跑得 10/10。阈值未改。
- 故障恢复 2/2：stale（409 `STALE_WORLD_STATE` → 重规划成功）、
  首次抓取错变体（`GRASP_MISSED` → 重规划成功）。
- 取消：`canceling→canceled`，ACK **0.217 s**，episode 不变；
  Worker 失联：`WORKER_LOST`，无重放；clean restart：新 Gateway → pick 成功。
- 资源：GPU 峰值 5147 MiB、关闭后 1422 MiB；worker RSS 峰值 8637 MiB。
- 交付：`README.md`、插件、启动脚本、可重放 `scripts\t09_eval.py` / `t09_restart_check.py`。

### 证据

- `artifacts\t09_2026-11-07_1435\summary.md`
- `eval2.json`（10/10）、`eval.json`（首次失败）、`reset_probe.json`
- `worker_lost_job.json`、`restart_pick.json`、`gpu_stats.json`、`worker_rss.json`
- `gateway*.db`/`log`、worker 输出目录

### 剩余问题

- 未运行随机场景集，泛化未声称；抓取变体仅两个位置。
- 相机单帧、ground-truth 控制；Warp 需预编译缓存。
- 新抓帧 DSH 工具已热重载注册；Gateway 单 worker、loopback、无认证。
- OP01（Newton）与 OP02（Python Server）为可选项，未开始。

## 运维记录 — 插件热重载（2026-11-07 16:07）

- 目的：让 T08 新增的 `robotics_capture_frame` / `robotics_get_frame` 注册到当前 Host。
- 操作：`plugin_manager` → `set_plugin(target="include:dsh-robotics", enabled=true)`，
  返回 `{"changed":false,"application":"applied","warnings":[]}`。
- 机制：DSH HMR 的 `reconcileProfile()` 比较已加载模块与磁盘（含 link 包目标），
  替换变化的 `index.js` 并重启该插件条目；Host 进程未重启。
- 验证：Host `Tool.listTools`（cordis_inspect）显示 14 个 robotics 工具，含
  `robotics_capture_frame`、`robotics_get_frame`；随后 `robotics_list_backends`
  经 Gateway（127.0.0.1:8714）真实返回，确认工具可执行。

## 增量 — Gateway GUI 控制台（2026-11-07）

- `/ui` 由 Gateway 进程提供：原生 HTML/CSS/JS 控制台，无构建步骤；新增只读
  `GET /api/v1/simulations`、`GET /api/v1/jobs`、`GET /api/v1/simulations/{id}/frames`。
- 功能：仿真启动/reset/close、抓帧、RGB/Depth（浏览器解析 `.npy` 并按 2%–98% 伪彩色）、
  世界状态（关节/末端/夹爪/对象/技能）、Jobs 与取消、事件流（WS）；
  `?depth=1` 可直接以 Depth 视图打开。
- 实测（真实 SO-101 相机 worker）：启动 92.3 s、抓帧 0.9 s；RGB PNG 与 depth npy 下载、
  帧列表与 UI 资源校验；Depth 0.4006–1.7674 m、valid 91.52%；全量 29 个测试通过。
- 证据：`artifacts\gui_2026-11-07\summary.md`、`console.png`、`console-depth.png`、
  `showcase\summary.json`、`smoke\summary.json`；冒烟脚本 `scripts\gui_smoke.py`。

## 增量 — Agent 监控地基 A1/A2/A3（2026-11-07）

### 目标与结论

按 `docs\agent-eval-prereqs.md` 的 A 阶段，让 Agent 在长技能运行期间仍能读状态、抓帧、取消。
改前 20 s `hold_pose` 期间 `/state` 阻塞 25.9 s、`/frames` 26.9 s、`/health` 16.9 s，
技能结束前所有请求排队；改后同一脚本：state p95 **23.9 ms**、frames max **62.7 ms**、
`/health` **15.7 ms**、cancel ACK **69 ms** 且最终 `canceled`。

### 关键实现

- **协作式技能执行**：`WorkerServer._run_skill` 在 sim 线程同步跑技能，每一步经 `step_cb`
  回调 `_hook` → `_service_commands`，在步与步之间回答 `observe`/`capture`；`sim.step`/render
  始终单线程。最初按清单做“独立线程”，实测技能线程在第 ~143 步（第一次 1 Hz 相机渲染）挂死
  （`camera.update()` 不能在非主线程调用），cancel 也无法 ACK；日志见
  `artifacts\monitor_probe\debug\`，该方案已放弃。
- **快照与帧缓冲**（`worker.py`：`SnapshotStore`、`WorkerMonitor`）：2 Hz `WorldState` 快照；
  1 Hz 相机帧进 8 帧环形缓冲，按需落盘；`capture(fresh=False)` 返回 `buffered: true`。
- **忙保护**：技能中第二个 `submit`、`reset` 返回 `WORKER_BUSY`；`close` 在技能结束后应答。
- **Gateway 非阻塞**（A2）：`get_state`/`reset`/`submit`/`capture`/`validate`/`execute` 走
  `run_in_threadpool`；`WORKER_BUSY` 映射为 409 可重试。
- **事件**（A3）：`store.events_for_job`、`GET /jobs/{id}/events`、
  `GET /simulations/{id}/frames/latest`；progress 改为每 10% 一条（修复每步重复发同一桶），
  单 job 上限 200 条；worker 在 submit 应答前发出的事件按 worker job id 缓冲后回放（修丢事件竞态）。
- **endpoint 复用**：`SubprocessEndpoint.close()` 不再置终态，`close_simulation` 后同一 Gateway
  可再次 `start_simulation`（去掉 T09 的“必须重启 Gateway”限制）。
- **`pick` 的 sim_step**：技能内实时镜像 `SO101PickPlace.step_count`，快照/帧的 `sim_step`
  不再等技能结束才跳变。
- **插件**：新增 `robotics_get_job_events`、`robotics_latest_frame`；热重载后 16 个工具在线。

### 验证

| 脚本 | 结果 |
|---|---|
| `scripts\monitor_probe.py`（20 s `hold_pose`，cancel） | 全通过；state p95 23.9 ms、frames max 62.7 ms、health 15.7 ms、cancel 69 ms |
| `scripts\monitor_probe.py`（真实 `pick`，监控中） | job succeeded；state p95 37 ms、frames max 51 ms、frame sim_step 0→300→…→1908 |
| `scripts\gateway_reuse_check.py`（close 后重启） | closed 202、restart succeeded（9.8 s）、`reused: true` |
| `plugin_manager` + `cordis_inspect` + 工具调用 | 16 个工具在线；两个新工具真实返回 |

- 单元测试 30 个（新增 `tests\unit\test_worker_monitor.py` 6 个：快照/帧缓冲/限流/并发命令循环），
  HTTP/WS 集成 7 个，全通过。
- 证据：`artifacts\monitor_probe\{before,after,pick,reuse,debug,debug2}\`、`plugin_reload.json`。

### 剩余问题

- 监控是协作式的：单步内的长阻塞（未来 VLA 推理）仍会推迟命令应答，策略需主动调用 `step_cb`
  或把推理切片。
- 相机仍为单目 640×480；C1（stub 策略）未开始。

## 增量 — Agent 评测 B1/B2/B3（2026-11-07）

### 实现

- `scripts\agent_eval.py`：
  - **B1 Rollout recorder**：每 episode 一份 JSONL（`artifacts\agent_eval\<run_id>\episode_NN.jsonl`），
    记录 reset/start、状态（2 Hz）、job 事件（5 Hz 增量，`after_seq` 游标）、帧（1 Hz，`buffered` 标记）、
    最终状态/结果/耗时；行结构与清单“建议字段”一致。
  - **B2 Eval runner**：读 JSON 矩阵、启动一次仿真、每 episode 前 `reset_simulation`；输出
    `report.json` + `report.md`（成功率、失败码/类别直方图、wall 统计、帧/事件计数）；`--verify`
    校验 report 与 JSONL 行数、事件序列、帧 `sim_step`。
  - **B3 失败分类**：`FAILURE_CATEGORIES` 把错误码映射为 9 类（抓偏/抓空/不可达/抓取位姿不可用/
    过期/忙/失联/超时/非法）；报告同时给 `failure_codes` 与 `failure_categories`。
- 矩阵：`configs\agent_eval_tasks.json`（10×pick）、`configs\agent_eval_failures.json`（2 失败 + 1 对照）。

### 实测

- 固定场景 10 次 pick：**10/10**，wall 14.46–14.91 s（mean 14.69、median 14.68），142 帧、150 事件；
  块最终 (0.2300–0.2305, 0.0577–0.0582, 0.0550)，在目标盒顶面；`verdict.matches_t09 = true`。
- 失败矩阵：x=0.16 → `GRASP_POSE_UNAVAILABLE`（0.37 s）；place_x=0.30 → `UNREACHABLE_TARGET`（0.18 s）；
  对照 → 成功；`expected_met` 3/3，类别 `{grasp_pose_unavailable: 1, unreachable: 1}`。
- `--verify`：baseline 579 行 / 150 事件 / 142 帧 / 247 状态；failure 84 行 / 23 事件 / 18 帧；0 问题。
- 测试：30 unit + 7 integration 仍全过。

### 证据

- `artifacts\agent_eval\baseline_pick_10\`（`report.json`、`report.md`、`episode_01..10.jsonl`、`report.partial.json`）
- `artifacts\agent_eval\failure_taxonomy\`（同上，3 个 episode）
- 矩阵与脚本：`configs\agent_eval_tasks.json`、`configs\agent_eval_failures.json`、`scripts\agent_eval.py`

### 剩余问题

- 评测路径用 `/skills`（与 T09 的 task/plan 路径不同，物理判定一致）；如需与 T07 的计划链严格对齐，
  可在矩阵里加 `task/plan` 模式。
- 场景种子固定（每次 reset 确定性相同），随机化场景尚未开始。
- D（补抓取变体）与 E（Agent 评测工作流）未开始。

## 增量 — 策略接入点 C1/C2（2026-11-07）

### 实现

- `stub_policy.py`（新）：`Policy` 协议（`reset`/`act`/`progress`）+ 两个脚本策略：
  `StubWaypointPolicy`（关节 chunk，approach→grasp→close→lift，带 60 步 settle）与
  `StubDeltaPolicy`（EE delta）；`PolicyFailure` 携带 `POLICY_TIMEOUT`/`NO_PROGRESS`。
- `so101_worker.py::_policy`：chunk 执行；EE delta → DLS（`root_physx_view.get_jacobians()`，
  阻尼最小二乘，显式固定到雅可比所在设备）；`max_seconds` 墙钟预算；关节/块连续
  `patience_chunks` 无变化 → `NO_PROGRESS`；取消在每个 chunk 边界检查。
- `server.py`：`phase_cb` → `progress` 事件（`data.phase` + `step`，4 Hz 限流）；
  `PolicyFailure` 映射为 FAILED + 具体错误码。
- 技能 `vla_stub` 注册进 RobotSpec（Gateway 可提交）。
- C2：每 chunk 一行 `policy_logs/policy_<ep>.jsonl`（chunk/sim_step/state_version/mode/targets/
  frame_id/latency_ms/progress），2 MB 轮转、单文件 5000 行上限。

### 实测（同一仿真，reset 后逐场景）

| 场景 | 结果 |
|---|---|
| 正常 lift | succeeded；block z 0.075→0.114（+39 mm）；100 chunks / 1000 steps / 7.6 s |
| fail=timeout, 5 s | failed `POLICY_TIMEOUT`，82 chunks / 820 steps |
| fail=no_progress | failed `NO_PROGRESS`（静止 15 chunks 后触发） |
| cancel t=1.5 s | canceled，ACK **21.8 ms** |
| mode=ee_delta | DLS 10 步无异常；未抓取 → `EMPTY_GRASP`（预期） |

期间 state p95 21–38 ms、帧 max 37–143 ms，监控通路未受影响。

### 证据

- `artifacts\policy_2026-11-07\summary.json` / `summary.md`；各场景 `summary.json`
- `worker_*\worker_*\policy_logs\policy_*.jsonl`（C2 action/obs 日志）

### 剩余问题

- stub 是脚本轨迹，不是视觉闭环；接真 VLA 只需替换 `act()`，obs 已含 RGB。
- `ee_delta` 的 DLS 用 palm 线速度雅可比，未做姿态控制；只验证接口，不用于抓取。
- E（Agent 评测工作流）未开始。

## 增量 — 抓取变体闭环 D（2026-11-07）

### 实现

- `scripts/so101_pick_sequence_solve.py` 新增 `--grasp-x/--grasp-y`（四个抓取 waypoint 随 x 求解，
  transfer/place/retreat 固定 0.22/0.05）与 `--preapproach-z`（高入口，用于全伸展变体）。
- 补解 x=0.16/0.17/0.19/0.21（starts=60，约 5 s/变体，每 waypoint 位置误差 0.00 mm）；
  `GRASP_VARIANTS` 变为 0.16–0.21 六个，容差保持 0.015。
- 0.21 直接接近会扫到块（`BLOCK_MOVED_ON_APPROACH`，8.4 mm > 8 mm），增加 z=0.175 高入口
  （位置误差 1.18 mm、方向误差 14.6°，仅转运点）；`_pick` 在有 `preapproach` 时先走该点。

### 闭环 demo（无模型训练）

| 步骤 | 结果 |
|---|---|
| 1. 块 x=0.16，未补变体 | failed `GRASP_POSE_UNAVAILABLE`（0.21 s） |
| 2. 补解并加入 4 个变体 | 位置误差 0.00 mm |
| 3. 重跑 x=0.16/0.17/0.19/0.21 | **4/4 passed**（wall 14.5–16.3 s） |
| 4. 默认 x=0.18 回归 | passed |

### 证据

- `artifacts\d_variants_2026-11-07\summary.md` / `summary.json`、`x016..x021.json`、
  `x021_preapproach.json`
- `artifacts\agent_eval\variant_fail\report.json`、`variants_open\report.json`、`baseline_recheck\report.json`

### 剩余问题

- 放置点仍固定 (0.22,0.05)；y 方向变体未补解。
- 0.21 的高入口是手工选择，通用化需要在求解器里加入“路径不扫块”的约束。

## 增量 — Agent 失败分析 E（2026-11-07）

### 输入

- 6 episode 混合矩阵 `configs\agent_eval_e.json` → `artifacts\agent_eval\e_failure_analysis\`
  （3 OK / `UNREACHABLE_TARGET` / `OBJECT_OUT_OF_WORKSPACE` / `GRASP_MISSED`）。

### Agent 分析（`analysis.md` / `analysis.json`）

- 复述分布与 `report.json` 完全一致（计数、类别）；额外给出根因：ep3 参数越界、ep4 场景越界
  （均预检拒绝、无运动）、ep5 强制 `grasp_x=0.16` 而块在 0.19（3 cm 偏移，失败帧确认夹爪在块左侧）。
- 建议：同场景不传 `grasp_x` 重跑；预测 succeeded、合并 pass_rate 4/6。
- 实际：`e_recheck` **succeeded**（块 (0.2287, 0.0581, 0.055)、xy 1.19 cm、16.0 s）。
- 差异记录：计数零差异；Agent 增加了根因与建议（脚本报告没有），作为 failure analyst 基线。

### 恢复实验（DSH 工具链，`recovery.json`）

- validate（hash `fce688…`）→ `move_object` 4 mm（state_version 0→1）→ execute
  **`STALE_WORLD_STATE`** → 新 validate（hash `932845…`）→ execute **task succeeded**
  （13.8 s，块 (0.2326, 0.0586, 0.055)）。旧计划没有被重放。

### 证据

- `artifacts\e_agent_analysis_2026-11-07\`（`analysis.md/json`、`recovery.json`、
  `grasp_missed_final.png`、`grasp_missed_frame.json`）
- `artifacts\agent_eval\e_failure_analysis\report.json`、`e_recheck\report.json`
- `configs\agent_eval_e.json`、`configs\agent_eval_e_recheck.json`

### 剩余问题

- 这是单轮、6 episode 的分析；跨 run 的趋势与自动建议生成未做。
- 真实 VLA 的失败模式（视觉误差、动作漂移）还没有样本。

