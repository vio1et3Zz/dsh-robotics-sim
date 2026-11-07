# T03 — Gateway、Job 与可确认取消（passed）

- 日期：2026-11-07（Asia/Hong_Kong）
- 运行目录：`artifacts\t03_2026-11-07_0210`
- 真实链路：DSH Gateway（FastAPI）→ 子进程 Isaac Worker → 真实 Franka PhysX
- 退出码：0（真实 smoke）；service tests 7/7；HTTP/WS tests 3/3

## 结论

- Gateway 能启动真实 Isaac Worker，`POST /simulations` 返回 202 + Job，约 8.0 s 后 Job `succeeded`，`GET /state` 返回真实 9 关节状态。
- `POST /simulations/{id}/skills` 提交 `calibrate_joint`（hold 1.5 s），Job 进入 `running`。
- `POST /jobs/{id}/cancel` 后 Worker 在 **0.124 s** 内停止并回 ACK，Job 变为 `canceled`；没有等满 1.5 s 的保持时间。
- 重复 cancel 返回 202，状态仍为 `canceled`；`state_version` 从 0 变为 1，说明停止后确实推进并保持了状态。
- `DELETE /simulations/{id}` 返回 202，Worker 与 Kit 进程退出；之后 `Get-Process python` 无残留。

## 实现的接口

| 接口 | 状态 |
|---|---|
| `GET /api/v1/health` | 实现（gateway/worker/simulation_ready 分开） |
| `GET /api/v1/backends` | 实现（kind/skills/alive；mock 明确 verified=false） |
| `POST /api/v1/simulations` | 实现（202 + 启动 Job） |
| `GET /api/v1/simulations/{id}` / `/state` | 实现 |
| `POST /api/v1/simulations/{id}/reset` | 实现（live Job 时 409） |
| `DELETE /api/v1/simulations/{id}` | 实现（live Job 时 409） |
| `POST /api/v1/simulations/{id}/skills` | 实现（内部/开发用；DSH 工具不会直接暴露） |
| `GET /api/v1/jobs/{id}` | 实现 |
| `POST /api/v1/jobs/{id}/cancel` | 实现（CANCELING → Worker ACK 后 CANCELED） |
| `WS /api/v1/events` | 实现（`after_seq` 补取） |
| `POST /api/v1/tasks` / `plans/validate` / `execute` | 实现基础版（顺序执行 plan steps） |

## 持久化与状态机

- SQLite：`jobs`、`events`、`idempotency`、`simulations`、`tasks`。
- Job 状态：`pending → running → succeeded / failed / canceled`，取消走 `canceling → canceled`。
- 取消只在 Worker `canceled` 事件到达后写 `canceled`；HTTP 请求本身不伪造完成。
- 同一幂等键重复提交返回同一 Job；真实 cancel smoke 中重复 cancel 未产生第二次动作。
- Gateway 重启时 `mark_live_jobs_lost()` 把未完成 Job 标为 `failed/WORKER_LOST`，不重放旧运动。
- `reset`/`close` 在存在 live Job 时返回 409 `CONTROL_BUSY`。

## 证据

- `summary.json`：真实 smoke 的完整请求/响应摘要、cancel 耗时、state_version。
- `service_tests.txt`：7 tests OK（幂等、取消 ACK、reset 竞争、stale episode、重启、worker 丢失）。
- `http_tests.txt`：3 tests OK（健康、完整 start→skill→cancel、WebSocket 补取）。
- `worker_1793984681\stdout.log` / `stderr.log`：真实 Kit 启动与关节表。
- `gateway.db`：Job/事件持久化。

## 已知限制

- Gateway 目前复用 Isaac 的 Python 环境（fastapi/uvicorn 已装），尚未拆独立 venv；不导入 Kit/Torch。
- 认证、TLS、多仿真实例、GUI timeline 暂停、相机帧都不在本阶段。
- plan 执行器只做顺序提交，预算/重规划在 T07 完善。
- `--headless` 仍会产生弃用警告；后续改用 `--viz none`。
- Worker 退出时 `worker_server_status.json` 偶尔来不及写（Kit 关闭时序），以 stdout/stderr 与 Gateway Job 记录为准。
