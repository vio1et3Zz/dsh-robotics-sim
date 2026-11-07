# 会话交接（2026-11-07 01:16）

## 一句话状态

T00–T03 已验收通过；T04 的 DSH 插件**已安装且工具可见**，但还没在真实 DSH 会话里跑完 start→skill→cancel→close 的演示。当前没有 Python/Kit 进程，也没有 Gateway 在监听。

## 已完成

| 阶段 | 状态 | 证据 |
|---|---|---|
| T00 基线/宿主接口 | passed | `artifacts\t00_2026-11-07_0015\baseline_manifest.json`、`doctor.json` |
| T01 协议/TS/fake | passed | `artifacts\t01_2026-11-07_0045\summary.json` |
| T02 真实 Franka PhysX | passed | `artifacts\t02_2026-11-07_0150\summary.json` |
| T03 Gateway/Job/取消 | passed | `artifacts\t03_2026-11-07_0210\summary.json` |

T03 的真实取消：hold 1.5 s 的 `calibrate_joint` 在 **0.124 s** 内停止并回 ACK，`state_version` 0→1；Gateway/Worker 进程已清理。

## T04 已完成的部分

- 插件源码：`packages\dsh-plugin\`
  - `index.js` 注册 12 个 `robotics_*` 工具；
  - `cordis.patch.yml` 插入 `id: dsh-robotics`；
  - `locale/en.json`、`locale/zh.json`、`icon.svg` 已补齐显示元数据。
- 插件安装已生效：
  - `%USERPROFILE%\.dsh\profiles\desktop\package.json` 含 `@local/dsh-robotics` 链接与 bundle；
  - 本 Agent 的 Tool 列表已能看到 `robotics_list_backends` 等工具；
  - `node --check packages\dsh-plugin\index.js` 通过。
- 没有手工改 profile 的 `package.json` / `cordis.patch.yml`。

## 恢复后的步骤

### 1. 确认环境干净

```powershell
Get-Process python -ErrorAction SilentlyContinue
Get-NetTCPConnection -LocalPort 8714 -State Listen -ErrorAction SilentlyContinue
```

### 2. 跑 Gateway 测试（可选但推荐）

```powershell
cd '<repo>'
& '<isaac-root>\env_isaaclab\Scripts\python.exe' -m unittest tests.unit.test_gateway_service tests.integration.test_gateway_http -v
```

### 3. 启动真实 Gateway（后台）

```powershell
$env:PYTHONPATH='<repo>\python'
& '<isaac-root>\env_isaaclab\Scripts\python.exe' -m robotics_gateway `
  --host 127.0.0.1 --port 8714 `
  --db '<repo>\artifacts\t04_2026-11-07_HHMM\gateway.db' `
  --worker isaac
```

第一个 `robotics_start_simulation` 会拉起 Kit，约 8–15 s；Worker 输出在 `artifacts\t04_...\worker_*\stdout.log`。

### 4. 在本 Agent 里直接调用工具

顺序建议：

1. `robotics_list_backends`
2. `robotics_start_simulation`（`profile_id=profile_dev_franka_physx`）
3. 轮询 `robotics_get_job_status` 直到 `succeeded`，拿到 `simulation_id`
4. `robotics_inspect_scene` / `robotics_get_robot_state`
5. `robotics_run_skill`：`skill=calibrate_joint`，参数 `{joint_name:"panda_joint1", delta_rad:0.1, hold_s:1.5}`
6. 轮询到 `running`，`robotics_cancel_job`
7. 轮询到 `canceled`，记录耗时
8. `robotics_close_simulation`
9. 把工具调用与结果写入 `artifacts\t04_...\summary.md`，更新 `PROGRESS.md` / `task_state.json`，T04 改 `passed`。

### 5. 插件安装状态的说明

- `plugin_manager install_bundle` 当时返回 “tool call aborted”，但 profile 已写入、工具已注册；**不要重复安装**。
- `Config.listConfigs` 显示 `status=absent`，因为插件没有导出 `Config` schema；工具可见即已加载。
- 若需要卸载：`plugin_manager remove_bundle` + target `@local/dsh-robotics`（会影响所有会话）。

## 关键路径与约定

- 项目：`<repo>`
- 插件：`packages\dsh-plugin`，Gateway 默认 URL `http://127.0.0.1:8714`（env `DSH_ROBOTICS_GATEWAY`）
- Gateway 包：`python\robotics_gateway`
- 真实 Worker：`python\robotics_backends\isaaclab_physx\server_entry.py`
- Franka 资产：`assets\isaac6\FrankaPanda\franka.usd`（远程资产在本机不可用，必须用本地镜像）
- 脚本执行策略：`powershell.exe -NoProfile -ExecutionPolicy Bypass -File ...`
- `--headless` 会打印弃用警告；可用 `--viz none`

## 已知问题（不阻塞 T04）

- Gateway 复用 Isaac Python 环境，尚未拆独立 venv。
- `worker_server_status.json` 偶尔因 Kit 关闭时序缺失；以 stdout/stderr + Job 记录为准。
- T04 工具调用会受插件默认 8714 端口限制；若换端口，需要给 Host 进程设置 `DSH_ROBOTICS_GATEWAY`（通常重启 DSH）。
- 真实端点目前只支持 Franka PhysX；SO-101 是 T05。
