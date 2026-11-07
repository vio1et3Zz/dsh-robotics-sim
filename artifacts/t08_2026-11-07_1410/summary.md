# T08 — 相机帧接口与最小 UI（passed）

- 时间：2026-11-07 14:10–14:29 (+08:00)
- 工程：`<repo>`
- 场景：`profile_so101_physx`，front camera 640×480，RGB + distance_to_image_plane

## 1. 实现

- `so101_objects_scene.py`：`SO101CameraSceneCfg` 增加
  - `front_camera`：pinhole 24 mm，horizontal_aperture 20.955，clip (0.1,20)，
    `data_types=["rgb","distance_to_image_plane"]`，world pose `(0.38,-0.42,0.30)`，
    quat xyzw `(-0.19947,0.12595,0.82169,0.51882)`，指向平台；
  - dome light 1500，改善渲染亮度。
- `so101_worker.py`：`capture_frame()`
  - 真实 RTX 渲染一帧，保存 `<worker>/frames/<frame_id>.png` 与 `.npy`；
  - 元数据：frame/camera id、simulation/episode、state_version、sim_step/sim_time、
    captured_at、尺寸、rgb/depth 格式、depth 单位 m 与类型、内参（fx/fy/cx/cy + 3×3）、
    外参（world 位置 + 四元数）、rgb/depth 范围、depth 无效比例、capture_ms、
    `ground_truth_scene=true`（画面由真实场景渲染，控制仍用 ground-truth 状态）。
- `server.py`：新增 `capture` op。
- `robotics_gateway`：`POST /api/v1/simulations/{id}/frames`、`GET /api/v1/frames/{id}`、
  `GET /api/v1/frames/{id}/rgb.png`、`GET .../depth.npy`、`GET /ui`（最小 UI：health/job/帧）。
- `packages/dsh-plugin`：新增 `robotics_capture_frame`、`robotics_get_frame`（源码 + `node --check` 通过）。
- 环境：Warp 相机 kernel 预编译到 ASCII 缓存
  `%USERPROFILE%\AppData\Local\Temp\dsh_robotics_warp`（`DSH_ROBOTICS_WARP_CACHE` 可覆盖）。
  中文/空格工作区路径会让 Warp 的 CUDA build 失败；DSH 文件沙箱也会阻断编译/PCH 临时文件，
  因此相机 worker 运行前需要预编译缓存（或全权限运行一次）。

## 2. 真实结果

| 项 | frame1（初始） | frame2（物块移到 x=0.20 后） |
|---|---|---|
| frame_id | `frame_e775abae…` | `frame_97739cbf…` |
| state_version | 0 | 1 |
| sim_step | 0 | 30 |
| RGB | 640×480×3 uint8，PNG，range [13,241] | 同尺寸；SHA256 与 frame1 不同 |
| Depth | 480×640×1 float32，范围 0.4006–1.7674 m | 范围一致；无效像素 8.47%（天空/背景） |
| 内参 | fx=fy=733.0，cx=320，cy=240 | 同 |
| 外参 | world (0.38,-0.42,0.30)，quat 同配置 | 同 |
| capture_ms | 0.47 | 0.61 |

- frame2 中红色积木明显右移，与 `move_object(0.20,0,0.075)` 和 state_version=1 一致。
- 帧文件通过 Gateway 的 `/frames/{id}/rgb.png`、`/depth.npy` 取回；`GET /frames/{id}` 返回元数据与 URL。
- `/ui` 返回 200（最小页面：health、capture、job、帧显示）。
- 资源：相机 worker 运行峰值 GPU **5022 MiB**（8 GiB 卡）；close 后回落到 **1430 MiB**，
  `worker_server_status.json = closed`，worker 进程退出；帧元数据在 close 后仍可读取。

## 3. 限制

- 只有前视相机；腕部相机未加（按计划先测前视）。
- 帧是真实场景的 RTX 渲染；控制/判定仍用 ground-truth 状态，未实现视觉闭环抓取，报告中明确标注。
- `robotics_capture_frame`/`robotics_get_frame` 已写入插件源码，但当前 DSH Host 会话未重载插件，
  工具尚未出现在本会话；Gateway HTTP 接口已实测。重载/重启 DSH 后可用。
- Warp 编译依赖预编译缓存；换机器/换 CUDA 版本需重新在可写 ASCII 路径编译。
- 640×480@按需 capture，未做视频流；每帧约 0.5 ms 读取（渲染在相机 update 内完成）。

## 4. 证据

- `camera_probe5.json`（全权限预编译/首帧）、`camera_probe6.json`（沙箱读取缓存）
- `frame1_meta.json`、`frame1_rgb.png`、`frame1_depth.npy`
- `frame2_meta.json`、`frame2_rgb.png`、`frame2_depth.npy`
- `gateway5.log`、`worker_1794032636\`（stdout/stderr、frames、worker_server_status.json）
- `gateway5.db`（frames 表）
