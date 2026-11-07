# Gateway GUI 控制台（Phase A）— 2026-11-07

## 目标

由 Gateway 进程自带一个可跑的操作台：启动/重置/关闭仿真、抓帧、RGB/Depth 视图、世界状态、
Jobs 与事件流。不依赖外部前端、无构建步骤。

## 实现

- 后端（`python/robotics_gateway/app.py`、`store.py`）：
  - 新增只读 `GET /api/v1/simulations`、`GET /api/v1/jobs`、`GET /api/v1/simulations/{id}/frames`；
  - `/ui` 返回 `python/robotics_gateway/ui/index.html`，`/ui/assets/*` 由 StaticFiles 提供，
    `/` 307 到 `/ui`。
- 前端（`ui/index.html`、`ui/style.css`、`ui/app.js`）：
  - 原生 HTML/CSS/JS；仿真列表与启动（三个固定 profile）、reset/close、抓帧、技能提交；
  - RGB 直接显示，Depth 在浏览器解析 `.npy`（float32 米制）并按 2%–98% 归一化伪彩色渲染；
  - 世界状态（episode/state_version、关节位置/速度、末端位姿、夹爪、对象、技能阶段）；
  - Jobs（最近 40 条、进度、取消、JSON 详情）；事件流复用 `/api/v1/events` WebSocket，
    断线按 `after_seq` 续传；
  - `?depth=1` 可直接以 Depth 视图打开（用于截图/演示）。
- 测试：`tests/integration/test_gateway_http.py` 新增 console 资源与列表接口用例；全量 29 个测试通过。

## 实测（真实 SO-ARM101 PhysX 相机 worker，本机）

`scripts/gui_smoke.py` 第一轮（8716，结束后 close）：

| 项 | 结果 |
|---|---|
| worker 启动 | 92.34 s |
| state | state_version 0，6 dof，`red_block` / `target_box` |
| 抓帧 | 0.96 s；RGB 152389 B PNG，depth 1228928 B npy（`\x93NUMPY`） |
| 帧列表 | 帧在列表中；`/ui` 与 `/ui/assets/app.js` 200 |
| close | `{"closed": true}`；之后 health `worker=dead` |

第二轮（8717，`--keep-open`，用于截图）：启动 92.32 s，抓帧 0.92 s；RGB sha256 `b39e63…`，
depth sha256 `17543c…`，与第一轮 depth 完全一致（同场景、seed 7）。
Depth 解析与帧元数据一致：min 0.4006 m / max 1.7674 m / valid 91.52%（invalid 8.48%）。

## 截图

- `console.png`：RGB 视图、世界状态、Jobs、事件流（1600×1200 窗口，headless Chrome）。
- `console-depth.png`：Depth 伪彩色（0.40–1.77 m，valid 92%），meta 行显示 depth 统计。

## 限制

- 页面只应绑定 Gateway 的 loopback 地址；Gateway 无鉴权，不要暴露到公网。
- 浏览器渲染验证使用本机 headless Chrome（命名管道需要一次性文件沙箱豁免）；未在其它浏览器/环境验证。
- mock profile 没有相机，抓帧返回 `UNSUPPORTED_CAPABILITY`（符合预期）。
- `close` 后当前 Gateway 进程的 worker 不可复用；再启动需重启 Gateway（与 README 一致）。

## 复现

```powershell
# 1) 启动 Gateway（真实相机 worker）
$env:PYTHONPATH='<repo>\python'
python -m robotics_gateway --host 127.0.0.1 --port 8717 `
  --db artifacts\gui_2026-11-07\showcase\gateway.db --worker isaac --device cuda:0

# 2) 启动 SO-101、抓帧、检查 UI 资源
python scripts\gui_smoke.py --base http://127.0.0.1:8717 `
  --out artifacts\gui_2026-11-07\showcase --keep-open

# 3) 浏览器打开 http://127.0.0.1:8717/ui
```
