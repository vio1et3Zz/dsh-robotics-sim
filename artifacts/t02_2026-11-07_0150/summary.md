# T02 — 真实 Franka PhysX Worker（passed）

- 日期：2026-11-07（Asia/Hong_Kong）
- 运行目录：`artifacts\t02_2026-11-07_0150`
- 命令：`powershell -ExecutionPolicy Bypass -File scripts\smoke_worker.ps1 -OutputDir <dir>`
- 退出码：0

## 结论

`AppLauncher → SimulationContext → InteractiveScene → Franka Articulation` 的真实链路已跑通：

- 真实 PhysX 场景启动，9 个关节（7 臂 + 2 指）全部读出实际名称、限位、速度限与 effort limit。
- `set_joint_position_target_index(full_data=True)` + `scene.write_data_to_sim()` + `sim.step(render=False)` + `scene.update(dt)` 的时序产生了可测量的物理响应，不是直接写状态。
- 非法 `delta_rad=10.0` 与未知关节在写入任何 buffer 前被拒绝；`sim_step` 与 `state_version` 均未变化。
- `reset()` 生成新 episode，`close()` 幂等，进程正常退出。

## 检查结果

| 检查 | 结果 |
|---|---|
| scene_ready | true |
| joint_count_matches | true |
| state_finite | true |
| illegal_action_rejected | true |
| unknown_joint_rejected | true |
| illegal_action_not_written | true |
| calibration_passed | true |
| state_version_advanced | true |
| episode_changed_on_reset | true |
| reset_state_valid | true |
| close_idempotent | true |

## 标定物理响应

- 关节：`panda_joint1`，限位 `[-2.8973, 2.8973]` rad，边距 0.05 rad。
- 起点：`-7.86e-05` rad；目标：`0.049921` rad；保持 50 step（0.25 s）后实测 `0.049896` rad。
- 保持误差：`2.57e-05` rad；尾段最大误差同为 `2.57e-05` rad。
- 回位后 `-0.001666` rad，回位误差 `0.00159` rad。
- 9 个关节位置全部为有限值。

## 机器人与资产

- 9 关节：`panda_joint1..7`、`panda_finger_joint1..2`。
- USD：本地镜像 `assets\isaac6\FrankaPanda\franka.usd`，SHA-256 `aa480a0c26cfacedd223259bd82b0825a44f857a27769db8be1d34c3f82c6d0f`，已写入 `runtime_manifest.json`。
- 资产来源：Isaac 6.0 S3 公共资产根；`panda_instanceable.usd` 已 404，`franka.usd` 可用。

## 时间

- Kit 启动到 manifest：约 3.5 s（另加进程启动）。
- 场景创建：2.42 s。
- 整个 entry：6.67 s。
- GPU：RTX 4060 Laptop，8188 MiB，驱动 616.92，`cuda:0`。

## 本阶段暴露并解决的问题

1. **默认 Franka 资产 404**：Isaac 6.0 不再托管 `panda_instanceable.usd`；改用 `FrankaPanda/franka.usd`。
2. **`omni.client.stat` 打不开远程 HTTPS**：即使 `urllib` 可取到文件，Kit 的远程 stat 仍失败；因此把 `FrankaPanda` 整棵树镜像到本地。
3. **默认地面引用远程 `default_environment.usd`**：改为程序化 `CuboidCfg` 地面，彻底去掉远程依赖。
4. **`--device` 与 AppLauncher 冲突**：从自定义参数里移除，改用 `app_launcher.device`。

## 限制

- 单环境、无物体、无相机；命令队列/取消/幂等属于 T03。
- `body_quat_w` 的 xyzw 顺序按 `root_quat_w` 的已核查约定处理，尚未用相机/标定独立验证。
- 远程 Nucleus 资产在本机不可用，T05 的 SO-ARM101 必须同样本地镜像并核对 LICENSE。
