# DSH Robotics

> **个人项目说明**：本仓库是在个人环境下开发使用的实验性工程，**未在其他环境验证**。路径、版本与
> GPU 假设按作者的 Windows 11 + RTX 4060 Laptop 8 GiB + Isaac Lab 3.0 / Isaac Sim 6.0.1 固定；
> 复现时请按“环境与依赖”一节调整，不要假定开箱即用。第三方资产不随仓库分发（见“资产”一节）。

DSH Host 插件 + 本地 Robotics Gateway + 真实 Isaac Lab PhysX Worker，提供
`robotics_*` 工具，让 DSH Agent 在真实物理仿真里启动场景、执行技能、规划任务并做
独立物理验证。T00–T09 与后续 A–E（监控地基、评测、策略接入点、变体闭环、失败分析）均在本机
完成并保存证据；SO-ARM101 的抓取/放置与相机均为真实仿真，mock/历史 Franka 结果不会计入
SO-ARM101 通过。

**English**: a personal, experimental DSH plugin + local FastAPI robotics gateway + real Isaac Lab
PhysX workers (SO-ARM101 / Franka). It exposes `robotics_*` tools to a DSH agent, stores jobs/events
in SQLite, supports task/plan validation with stale-state rejection, skill monitoring during long
runs, and a JSONL-based evaluation runner. Developed on Windows 11 with Isaac Lab 3.0 / Isaac Sim
6.0.1; other environments are untested. Third-party robot assets are not redistributed.

## 目录

```
dsh-robotics/
  packages/dsh-plugin/            DSH Host 插件（robotics_* 工具）
  packages/robotics-protocol/     TS 类型 + Ajv 校验（由 Pydantic 导出）
  python/robotics_protocol/       协议单一源（Pydantic v2）
  python/robotics_backends/       fake + isaaclab_physx（Franka dev / SO-ARM101 worker）
  python/robotics_gateway/        FastAPI Gateway、SQLite jobs/events/tasks/frames
  python/robotics_gateway/ui/     Gateway 自带操作台（原生 HTML/CSS/JS，无构建步骤）
  configs/                        Agent 评测任务矩阵（JSON）
  embodiments/so_arm101/          robot_spec / joint_mapping / scene 配置
  scripts/                        doctor、smoke、eval、离线运动学/相机探针
  docs/execution/                 PROGRESS、task_state、eval_manifest、HANDOFF
  docs/agent-eval-prereqs.md      Agent 评测与数据闭环的前置工作清单
  artifacts/                      小型验收证据（帧/DB/日志已 gitignore）
```

资产目录 `assets/isaac6/` 不随仓库分发，按“资产”一节放置后即可运行。

## 环境与依赖

**作者本机（唯一验证过的环境）**

- Windows 11, RTX 4060 Laptop 8 GiB, CUDA 12.8
- Isaac Lab `v3.0.0-beta2.patch1-dirty`（commit `ffff603e…`），Isaac Sim `6.0.1.0`
- Python 3.12.10（Isaac Lab 的 env，Torch 2.11.0+cu128）
- Node.js 24（仅 `packages/robotics-protocol` 的 schema 校验/类型检查需要）

Gateway 侧 Python 依赖（`requirements.txt`，版本为实测）：

```powershell
pip install -r requirements.txt
# fastapi==0.120.4 uvicorn==0.29.0 pydantic==2.11.10 numpy==2.3.1 pillow==12.2.0 scipy==1.17.0
```

Worker 直接在 Isaac Lab 的 Python 环境里运行，不需要把本仓库安装成包；Gateway 通过
`PYTHONPATH` 注入 `python/`。

**环境变量（均有默认值，按需覆盖）**

| 变量 | 用途 | 默认 |
|---|---|---|
| `DSH_ISAAC_PYTHON` | 启动 Isaac worker 的解释器 | `--python` / 当前解释器 |
| `DSH_ISAACLAB_ROOT` | Isaac Lab checkout（仅 T02 provenance） | `~/isaac/IsaacLab` |
| `DSH_ISAACLAB_SOURCE` | Isaac Lab 源码树（未安装进 worker 解释器时用） | `~/isaac/IsaacLab/source/isaaclab`（存在才用） |
| `DSH_ROBOTICS_WARP_CACHE` | Warp 相机 kernel 预编译缓存（必须是 ASCII 可写路径） | `%TEMP%\dsh_robotics_warp` |
| `DSH_ROBOTICS_GATEWAY` | 插件访问的 Gateway 地址 | `http://127.0.0.1:8714` |

中文/空格工作区下 Warp 的 CUDA 构建会失败，所以缓存路径必须显式指到 ASCII 目录；预热一次后
后续相机 worker 启动会明显变快（约 90 s → 10 s）。

### 资产（不随仓库分发）

第三方资产（Franka、SO-ARM101、Materials）各自 Apache-2.0，把以下路径补齐即可运行
（来源与许可证见 `THIRD_PARTY_NOTICES.md`）：

```
assets/isaac6/FrankaPanda/franka.usd
assets/isaac6/RobotStudio/so101_new_calib/so101_new_calib.usd
assets/isaac6/Materials/...
```

来源：Isaac Lab 的 `isaaclab_assets`（Franka）与 RobotStudio 的 SO-ARM101 USD。本仓库只保留
路径与许可证说明，不重新分发这些二进制；路径可按需改为其他本地镜像。

## 快速开始

```powershell
# 1) 依赖
pip install -r requirements.txt

# 2) 启动 Gateway（Isaac worker 用 Isaac Lab env 里的解释器）
$env:PYTHONPATH = (Resolve-Path .\python).Path
python -m robotics_gateway --host 127.0.0.1 --port 8714 `
  --db artifacts\gateway.db --worker isaac --device cuda:0 `
  --python 'C:\path\to\env_isaaclab\Scripts\python.exe'

# 3) 浏览器打开控制台
#    http://127.0.0.1:8714/ui

# 4) 测试（不需要 GPU）
python -m unittest discover -s tests/unit -v
python -m unittest tests.integration.test_gateway_http -v
```

Gateway 会按 `SimulationSpec` 的固定 profile 启动对应 worker（Franka dev 或 SO-ARM101），
不执行客户端传入的可执行文件。首次启动 SO-ARM101 相机 worker 约 90–100 s（RTX/着色器缓存冷启动），
缓存热后约 10 s。

安装 DSH 插件：把 `packages/dsh-plugin` 作为本地 link 包加入当前 DSH profile，然后用
`plugin_manager set_plugin(target="include:dsh-robotics", enabled=true)` 注册 16 个工具；
包内自带 `cordis.patch.yml`，可参考该文件接入自己的 profile。

## GUI 控制台

启动 Gateway 后打开 <http://127.0.0.1:8714/ui>（`/` 会 307 重定向到 `/ui`）。页面由同一个
Gateway 进程提供，复用其 REST/WS API，无外部前端依赖、无构建步骤。

- **仿真**：启动（SO-101 / Franka dev / mock，含 seed 与 SimulationSpec 预览）、reset、close、抓帧
- **相机**：前视 RGB；Depth 在浏览器端解析 `.npy`（float32 米制）并按 2%–98% 归一化伪彩色渲染
- **世界状态**：episode / state_version、关节位置与速度、末端位姿、夹爪、对象、技能阶段
- **Jobs**：最近 40 条、进度、取消（等待 worker 确认）、点击查看完整 JSON
- **事件流**：复用 `/api/v1/events` WebSocket，断线自动重连（`after_seq` 续传）
- `?depth=1` 直接以 Depth 视图打开；本地证据见 `artifacts/gui_2026-11-07/`（含两张截图）

新增只读接口：`GET /api/v1/simulations`、`GET /api/v1/jobs`、`GET /api/v1/simulations/{id}/frames`。
页面只应绑定 Gateway 的 loopback 地址；Gateway 无鉴权，不要暴露到公网。

冒烟脚本（需先启动 Gateway，`--keep-open` 便于截图）：

```powershell
python scripts\gui_smoke.py --base http://127.0.0.1:8714 --out artifacts\gui_smoke --keep-open
```

## DSH 工具（16 个）

`robotics_list_backends`、`start_simulation`、`inspect_scene`、`get_robot_state`、
`run_skill`、`get_job_status`、`get_job_events`、`cancel_job`、`reset_simulation`、
`close_simulation`、`create_task`、`validate_plan`、`execute_task`、`capture_frame`、
`get_frame`、`latest_frame`。

- `robotics_start_simulation(profile_id="profile_so101_physx")` 启动 SO-ARM101（含前视相机）。
- `robotics_run_skill(skill, parameters)`：开发/故障注入接口；正式流程用 task/plan。
- `robotics_create_task` → `robotics_validate_plan` → `robotics_execute_task`：
  validate 时保存世界快照，execute 时若 episode/state_version 或对象位置（>5 mm）变化，
  返回 409 `STALE_WORLD_STATE`。
- `robotics_capture_frame` / `robotics_get_frame`：前视 RGB/Depth 帧与元数据。
- `robotics_get_job_events(job_id, after_seq)`：读单个 job 的 created/started/progress/
  succeeded/failed/canceled 事件时间线；`next_seq` 作为下一次的游标，分页不重不漏。
- `robotics_latest_frame(simulation_id)`：取该仿真最近一帧（不重新渲染），适合技能运行中轮询。

> 插件源码更新后，用 `plugin_manager` 的 `set_plugin` 触发热重载或重启 DSH Host 即可注册新工具；
> 本次已验证热重载生效（16 个工具在线，`toolCount` 见 `artifacts/monitor_probe/plugin_reload.json`）。

## SO-ARM101 技能

- `observe_scene`：6 关节状态、嘴部 EE、`red_block`/`target_box` 对象。
- `pick`：顶抓固定轨迹；抓取变体 x=0.16–0.21（6 个，每 1 cm，由实测块位选择最近变体）；
  夹爪嘴部坐标由碰撞网格标定；
  阶段谓词：抓取居中、块随嘴抬升、转移、放置区域、释放。失败码：`GRASP_MISSED`、`EMPTY_GRASP`、
  `UNREACHABLE_TARGET` 等。
- `move_object`（开发/故障注入，仅 red_block）、`hold_pose`（取消测试）。
- `vla_stub`：脚本策略（C1），Policy 协议、action chunk、`phase` 事件、超时/无进展注入；
  可替换为一个真实 VLA 的 `act()`。

## 技能运行中的监控（A1/A2/A3）

技能仍在 sim 线程执行，但**每一步之后回调命令循环**（协作式调度），`sim.step`/render 始终只在一个
线程里（Isaac 渲染器有线程亲和：把技能放独立线程时，第一次 1 Hz `camera.update()` 就会挂死）：

- 每步之后发布快照（2 Hz）、渲染缓冲帧（1 Hz，最近 8 帧，按需落盘 PNG/NPY）；
- 技能中 `observe` 返回最新快照，`POST /frames` 返回缓冲帧（`buffered: true`），空闲时才现渲；
  `sim_step` 随快照与帧递增；
- 第二个 `submit` 返回 `WORKER_BUSY`，技能运行中 `reset` 同样被拒绝；
- Gateway 的 worker 调用改走线程池（A2），技能期间 `/health`、WS、GUI 不再冻结；
- 新接口：`GET /api/v1/jobs/{job_id}/events?after_seq=&limit=`、
  `GET /api/v1/simulations/{id}/frames/latest`；progress 每 10% 一条，单个 job 上限 200 条。

验收脚本（需先启动 Gateway）：

```powershell
python scripts\monitor_probe.py --base http://127.0.0.1:8714 `
  --skill hold_pose --parameters '{\"duration_s\": 20}' --cancel-at 8 --health-at 10 `
  --out artifacts\monitor_probe\after
```

改前（`artifacts/monitor_probe/before/`）：20 s 技能期间 `/state` 阻塞 **25.9 s**、`/frames`
**26.9 s**、`/health` **16.9 s**，技能结束前所有请求都排队。
改后（`artifacts/monitor_probe/after/`，同一 20 s `hold_pose`）：state p95 **23.9 ms**、frames
max **62.7 ms**、`/health` **15.7 ms**、cancel ACK **69 ms** 且最终 `canceled`、frame `sim_step`
0→315→653→982→1331 递增。真实 `pick` 在监控下仍成功（`artifacts/monitor_probe/pick/`），
close 后可在同一 Gateway 重启仿真（`artifacts/monitor_probe/reuse/`）。

## Agent 评测（B1/B2/B3）

`scripts/agent_eval.py` 按 JSON 任务矩阵跑 episode，并在运行中记录：

- 每个 episode 一份 JSONL（`artifacts/agent_eval/<run_id>/episode_NN.jsonl`）：reset/start、状态采样
  （2 Hz）、job 事件（5 Hz 增量读取）、帧（1 Hz，带 `buffered`）、最终结果与耗时；
- `report.json` + `report.md`：成功率、失败码与失败类别直方图、wall 时间、帧/事件计数；
- `--verify <run>` 校验 `report.json` 与 JSONL 行数/事件序列/帧 `sim_step` 一致。

```powershell
python scripts\agent_eval.py --base http://127.0.0.1:8714 `
  --matrix configs\agent_eval_tasks.json --out artifacts\agent_eval\baseline_pick_10
python scripts\agent_eval.py --verify artifacts\agent_eval\baseline_pick_10
```

- 固定场景 10 次 pick：**10/10**（wall 14.46–14.91 s、mean 14.69；142 帧、150 事件），与 T09 的
  10/10 对齐；runner 走 `/skills`，T09 走 task/plan/execute，物理场景与判定相同。
- 失败矩阵 3 例：`GRASP_POSE_UNAVAILABLE`（x=0.16）、`UNREACHABLE_TARGET`（place_x=0.30）、
  成功对照，3/3 与 `expected_failure` 一致；类别直方图 `{grasp_pose_unavailable: 1, unreachable: 1}`。
- 矩阵：`configs/agent_eval_tasks.json`、`configs/agent_eval_failures.json`、
  `configs/agent_eval_e.json`、`configs/agent_eval_variants.json`；证据见下一节。

Agent 失败分析（E）：对 6 个混合 episode 的 `report.json`，Agent 复述的计数/类别与脚本完全一致，
并从失败帧与 JSONL 给出根因（预检越界 2 例、强制 `grasp_x` 偏移 1 例），建议“同场景不传 `grasp_x`
重跑”——实际重跑 succeeded（合并 pass_rate 3/6 → 4/6）。恢复实验用 DSH 工具链：
validate → `move_object` → execute `STALE_WORLD_STATE` → 重新 validate → execute succeeded。
证据：`artifacts/e_agent_analysis_2026-11-07/`（`analysis.md/json`、`recovery.json`、失败帧）。

## 策略接入点（C1）

`vla_stub` 技能用与真实 VLA 相同的协议跑脚本策略（不加载模型）：

- `Policy`：`reset()` / `act(obs) -> action chunk`（`stub_policy.py`）；obs 含关节、末端、夹爪、
  对象与前视帧 id（RGB 数组直接传入）；action 为关节目标 chunk，或 EE delta（worker 用 DLS 转关节）；
- 事件：`progress` 事件带 `phase`（policy/execute/verify）、`step`；每个 chunk 一行
  `policy_logs/policy_<ep>.jsonl`（chunk/sim_step/state_version/mode/targets/frame_id/latency_ms）；
- 预算：`max_seconds` → `POLICY_TIMEOUT`；关节/块连续 15–20 chunk 无变化 → `NO_PROGRESS`；
  取消在 chunk 边界检查。

实测（`artifacts/policy_2026-11-07/summary.md`）：正常 lift **通过**（块抬升 39 mm、100 chunks、7.6 s）；
timeout → `POLICY_TIMEOUT`（~5 s）；no_progress → `NO_PROGRESS`；取消 → `canceled`（ACK **21.8 ms**）；
`ee_delta` 的 DLS 路径执行 10 步无异常。

## 验收与证据

| 阶段 | 内容 | 证据 |
|---|---|---|
| T00 | 基线/宿主接口 | `artifacts/t00_*/baseline_manifest.json`、`doctor.json` |
| T01 | 协议/TS/fake | `artifacts/t01_*/summary.json` |
| T02 | 真实 Franka PhysX | `artifacts/t02_*/summary.json` |
| T03 | Gateway/Job/取消 | `artifacts/t03_*/summary.json` |
| T04 | DSH 插件/首次真实调用 | `artifacts/t04_*/summary.md` |
| T05 | SO-101 资产/标定 | `artifacts/t05_*/summary.md` |
| T06 | 抓取/放置物理验证 | `artifacts/t06_*/summary.md`（3/3 一致，抬升 43 mm） |
| T07 | Agent 计划/过期/恢复 | `artifacts/t07_*/summary.md`、`dsh_tools.md` |
| T08 | 相机帧/UI | `artifacts/t08_*/summary.md`（640×480 RGB+depth） |
| T09 | 固定验收 | `artifacts/t09_*/summary.md`、`eval.json`、`eval_manifest.json` |
| A1–A3 | 技能中监控 | `artifacts/monitor_probe/{before,after,pick,reuse}/`、`plugin_reload.json` |
| B1–B3 | Agent 评测 | `artifacts/agent_eval/baseline_pick_10/report.md`、`failure_taxonomy/report.md` |
| C1–C2 | 策略接入点 | `artifacts/policy_2026-11-07/summary.md`（5 场景 + policy 日志） |
| D | 抓取变体闭环 | `artifacts/d_variants_2026-11-07/summary.md`；`agent_eval/variant_fail|variants_open/` |
| E | Agent 失败分析 | `artifacts/e_agent_analysis_2026-11-07/analysis.md`、`recovery.json`、`e_recheck/report.json` |

### T09 验收摘要

- 固定场景 10 次 pick/place：**10/10 成功**（`eval2.json`；每次 wall ~15 s、sim 10.3–13.05 s）。
- 第一次 eval 1/10 暴露 reset 缺陷（保留为失败证据）；修复后复跑，阈值未改。
- 故障恢复 2/2：`STALE_WORLD_STATE` 后重规划成功、`GRASP_MISSED` 后重规划成功。
- 取消 ACK **0.217 s**；Worker 失联 `WORKER_LOST`；clean restart 后 pick 成功。
- GPU 峰值 **5147 MiB**，关闭后 **1422 MiB**；worker RSS 峰值 8637 MiB。
- 随机场景泛化未运行、未声称。

## 已知限制

- SO-ARM101 有 6 个已求解抓取变体（x=0.16–0.21，每 1 cm）但放置点固定 (0.22,0.05)；任意目标
  需要离线求解器重新生成路点（`scripts/so101_pick_sequence_solve.py --grasp-x <x>`）。
- 相机为前视单目 640×480；技能运行时 1 Hz 进缓冲、空闲时按需单帧。控制/判定使用 ground truth，
  未声称视觉闭环抓取。
- 评测抓帧会落 PNG/NPY 到 worker 目录（10 次 pick ≈ 200 MB，320 个文件）；只要 JSONL 时间线时
  可清理 `worker_*/frames/`。
- Gateway 仅 loopback、单 worker、无认证；`close_simulation` 后可再次 `start_simulation`
  （2026-11-07 修复 endpoint 复用，证据 `artifacts/monitor_probe/reuse/`）。
- 三方资产不随仓库分发，需自备 `assets/isaac6/...`；其他 Isaac Lab 版本未验证。
- Kit 启动偶发挂起（尤其首次 RTX 初始化）；重试即可，证据中保留失败记录。
- `move_object` 是故障注入技能，不属于正常运行策略。

## 诊断

```powershell
powershell -ExecutionPolicy Bypass -File scripts\doctor.ps1
python -m unittest tests.unit.test_gateway_service tests.integration.test_gateway_http -v
```

详细阶段记录见 `docs/execution/PROGRESS.md`，固定验收清单见 `docs/execution/eval_manifest.json`，
后续 Agent 评测与数据闭环的前置工作见 `docs/agent-eval-prereqs.md`。

## License

- 本仓库代码：MIT，见 `LICENSE`。
- 第三方资产（Franka、SO-ARM101、Materials）、Isaac Lab / Isaac Sim、Python/Node 依赖：各自许可证，
  见 `THIRD_PARTY_NOTICES.md`；机器人资产与 Isaac 二进制不随本仓库分发。
