# T04 — DSH 插件与真实工具闭环（passed）

- 日期：2026-11-07（Asia/Hong_Kong）
- 运行目录：`artifacts\t04_2026-11-07_1150`
- 工具来源：`@local/dsh-robotics`（已装入 desktop profile）
- Gateway：`127.0.0.1:8714`，`--worker isaac`

## 结论

本 Agent 直接调用 `robotics_*` 工具完成了真实闭环：

1. `robotics_list_backends` → `isaaclab_physx`，技能 `calibrate_joint` / `hold_pose` / `observe_scene`。
2. `robotics_start_simulation(profile_dev_franka_physx)` → Job `job_f9a2f4df...`，6.51 s 后 `succeeded`，simulation `sim_5d52ca7c...`。
3. `robotics_inspect_scene` → `state_version=0`，9 关节，状态全为有限值。
4. `robotics_run_skill(hold_pose, duration_s=20)` → Job `job_76e677a2...`。
5. `robotics_cancel_job` → 进入 `canceling`（progress 0.40975）；46 ms 后 Worker ACK，Job 变 `canceled`。
6. 取消后 `robotics_inspect_scene` → `state_version=1`，`sim_time_s=8.58`，技能状态 `canceled / "cancel acknowledged"`。
7. `robotics_close_simulation` → `closed=true`；Worker 写 `worker_server_status.json`（`status=closed`）并退出。

## 关键时间

| 事件 | 时间（UTC） |
|---|---|
| 启动 Job 创建 | 03:10:15.166 |
| 启动 Job 成功 | 03:10:21.678 |
| hold_pose Job 创建 | 03:10:45.212 |
| cancel 请求（Gateway 记录） | 03:10:51.904 |
| 取消完成（Worker ACK） | 03:10:51.950 |
| Worker 关闭状态写入 | 03:11:13.849 |

取消 ACK 延迟约 **46 ms**；hold 目标 20 s，实际在约 6.7 s 时停止。

## 证据

- `gateway_run.json`：2 个 Job、205 个事件、1 个 simulation。
- `worker_1794020999\stdout.log`、`stderr.log`、`worker\worker_server_status.json`。
- `gateway.db`：Job/事件持久化。
- 本次会话中的工具调用与返回（`robotics_list_backends`、`start`、`inspect`、`run_skill`、`cancel`、`close`）。

## 本阶段修复的真实缺陷

1. **Worker 存活判断错误**：`env_isaaclab\Scripts\python.exe` 是 shim，Popen 跟踪的父进程会先退出；改为按已接受的 TCP 连接状态判断 `alive()`，避免重复 spawn。
2. **Worker socket 超时**：`create_connection(timeout=10)` 让 Worker 侧 socket 保持 10 s 超时，空闲后读线程报错并断开；连接后改回阻塞模式。
3. **Kit 关闭挂起**：Windows 上 `simulation_app.close()` 可能长时间不返回；Worker 现在先写关闭状态，再有界等待 10 s，最后 `os._exit`，保证进程退出。
4. **短技能来不及取消**：模型工具往返 > 4 s，新增有界 `hold_pose`（最长 30 s），用于真实取消演示。

## 限制

- `robotics_*` 工具默认访问 `http://127.0.0.1:8714`；换端口需给 Host 设置 `DSH_ROBOTICS_GATEWAY`。
- `robotics_run_skill` 是开发/内部接口；正式任务用 task/plan 工具（T06/T07 完善）。
- 插件安装后 `Config.listConfigs` 显示 `status=absent` 是因为未导出 `Config` schema，不代表未加载；工具可见即已生效。
