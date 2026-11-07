# T09 — 固定工程验收与交付（passed）

- 时间：2026-11-07 14:32–14:56 (+08:00)
- 工程：`<repo>`
- Manifest（运行前固定）：`docs\execution\eval_manifest.json`
- 场景：`profile_so101_physx`，SO-ARM101 + 红块 + 目标盒 + 前视相机

## 1. 固定验收结果（eval2，修正 reset 后）

| 指标 | 结果 |
|---|---|
| 固定场景 10 次 pick/place | **10/10 成功**（`successes=10`, `pass_rate=1.0`） |
| 成功判定 | job `succeeded` 且块最终在目标盒顶面（XY ≤ 5 cm，z ≤ 0.09 m） |
| 每次 wall time | 14.9–15.9 s |
| 每次 sim time | 10.3–13.05 s |
| 终态块 | 例如 (0.2302, 0.0582, 0.0550) |
| 阈值调整 | **无**；manifest 在运行前写入，未按结果改阈值 |
| 随机场景泛化 | 未运行，未声称 |

> 第一次 eval（`eval.json`）只 1/10：`reset` 未恢复物体初始位姿，导致第 2 次起
> `OBJECT_OUT_OF_WORKSPACE`。这是**失败证据**，保留；随后定位 `RigidObject.reset` 只清外力、
> 不恢复 root pose，worker 改为显式恢复初始块位姿/关节再 settle（`reset_probe.json`：两次
> reset 后块误差 <1 mm、关节 <0.011 rad），再跑 `eval2` 得 10/10。没有修改成功阈值。

## 2. 故障恢复（两类）

| 用例 | 注入 | 结果 |
|---|---|---|
| `stale_after_object_move` | validate 后把块移 4 mm | 旧计划执行 **409 STALE_WORLD_STATE**；重观察、重规划后 succeeded，块 (0.2377,0.0597,0.0550) |
| `first_grasp_wrong_variant` | 块移到 x=0.20，首次 plan `grasp_x=0.18` | 首次 job **failed GRASP_MISSED**（块未被扰动）；重观察、`grasp_x=0.20` 后 succeeded，块 (0.2310,0.0581,0.0550) |

两次恢复都在 task budget（180 s sim / 240 s wall、6 次技能、2 次重试）内；恢复成功不覆盖首次失败记录。

## 3. 取消 / 失联 / clean restart

| 测试 | 结果 |
|---|---|
| 取消 pick（1.5 s 后） | `canceling → canceled`，Worker ACK **0.217 s**；episode 不变；state_version 递增；块仍在初始位（0.18,0,0.075） |
| Worker 失联（hold 中杀 worker 进程） | live job **failed `WORKER_LOST: worker exited while the job was live`**，无重放（`worker_lost_job.json`） |
| clean restart（停 Gateway/worker → 新 Gateway → 新 sim） | start succeeded，task/plan pick succeeded，块 (0.2302,0.0584,0.0550)，wall 14.5 s（`restart_pick.json`） |
| 关闭清理 | worker `worker_server_status=closed`，进程退出；GPU 4892→1422 MiB |

## 4. 资源与时序

| 指标 | 值 |
|---|---|
| GPU 内存（相机 worker，eval 窗口） | 峰值 **5147 MiB**，中位 4896 MiB（530 个活跃采样） |
| GPU 关闭后 | **1422 MiB**（桌面基线 ~1.2–1.4 GiB） |
| Worker RSS（hold 期间） | 峰值 **8637 MiB**（10 次采样） |
| 每次 pick | wall ~15 s，sim ~10.3–13.1 s，约 2060–2610 sim steps |
| 相机帧 | 640×480 RGB + 480×640 depth，depth 0.4006–1.767 m，~8.5% 无效像素 |
| 本会话 DSH 工具调用 | 约 24 次；失败/重试：抓帧工具未注册 1 次、worker 启动挂起重试 3 次；无人工改阈值 |
| 约束违反 | 未发现；joint targets 均经限位裁剪，plan 只接受后端声明的 skills |

## 5. 交付

- README：`README.md`（安装、启动、14 个工具、技能、验收表、限制）。
- 插件：`packages\dsh-plugin`（`node --check` 通过；已装入 desktop profile；新增抓帧工具需重载 Host）。
- Gateway/Worker 启动：`scripts\start_gateway.ps1`、`scripts\smoke_worker.ps1`、`scripts\smoke_so101.ps1`。
- 验收清单：`docs\execution\eval_manifest.json`；阶段记录：`docs\execution\PROGRESS.md`。
- 可重放：`scripts\t09_eval.py`、`scripts\t09_restart_check.py`、`scripts\so101_reset_probe.py`。

## 6. 最终证据链

1. 真实 DSH 会话：T07 `dsh_tools.md`（Agent 由 inspect_scene 生成 task/plan；stale + GRASP_MISSED 恢复）。
2. 真实 SO-ARM101 PhysX：10/10 pick/place（`eval2.json`），3/3 早期一致 run（T06）。
3. 物理后置条件：块在盒顶的实测位姿；照片 `frame1/frame2`（相机为场景渲染，控制用 ground truth）。
4. 完整失败/恢复链：`eval.json`（reset 缺陷失败）→ `reset_probe.json`（修复验证）→ `eval2.json`（10/10）；
   `worker_lost_job.json`、`restart_pick.json`。
5. 资源：`gpu_stats.json`、`worker_rss.json`。

## 7. 限制（最终）

- 固定场景 10 次验收达标；未运行预先声明的随机场景集，泛化未声称。
- 抓取变体仅 x=0.18/0.20，place 固定 (0.22,0.05)；任意目标需离线求解器重算。
- 相机前视单帧；Warp 需预编译 ASCII 缓存；控制/判定为 ground truth，非视觉闭环。
- Gateway loopback、单 worker、无认证；close 后同进程不可再启动（clean restart 用新进程）。
- `robotics_capture_frame`/`robotics_get_frame` 已写入插件源码，需重载/重启 DSH Host 才会注册。
- Kit/RTX 首次启动偶发挂起；已保留失败记录并重试成功。
