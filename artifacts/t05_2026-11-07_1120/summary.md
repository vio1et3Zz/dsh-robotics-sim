# T05 — SO-ARM101 资产、标定与场景（passed）

- 日期：2026-11-07（Asia/Hong_Kong）
- 运行证据：`artifacts\t05_2026-11-07_1120`
- 资产镜像：`assets\isaac6\RobotStudio\so101_new_calib`

## 资产核验

- S3 前缀：`Assets/Isaac/6.0/Isaac/Robots/RobotStudio/so101_new_calib/`
- 本地镜像：6 个文件、23,256,986 字节（剔除 `.thumbs`），含 `LICENSE`（Apache-2.0）。
- 主 USD：`so101_new_calib.usd`，1,464 B，SHA-256 `04ef30434ff8bb752241ce5c6d33304ad04629b5d9290c580c4fd8b9fc89efb3`。
- 依赖层：`configuration/so101_new_calib_base.usd`（23.2 MB）及 physics/robot/sensor 配置。
- 加载失败时 `so101_scene.py` 直接抛 `ASSET_NOT_READY`，不会静默换成 Franka。

## 关节与映射

USD 中 5 个臂关节 + 1 个 **旋转**夹爪关节（USD 限位为度，Isaac Lab 读取为弧度）：

| 关节 | 限位 (rad) | home | 方向 |
|---|---|---|---|
| shoulder_pan | -1.9199..1.9199 | 0.0 | +1 |
| shoulder_lift | -1.7453..1.7453 | 0.0 | +1 |
| elbow_flex | -1.6900..1.6900 | 0.0 | +1 |
| wrist_flex | -1.6581..1.6581 | 0.0 | +1 |
| wrist_roll | -2.7438..2.8412 | 0.0 | +1 |
| gripper | -0.1745..1.7453 | 0.0 | +1 |

- 所有关节 `logical_name == asset_name`，索引 0..5 唯一。
- 命令 `+0.05 rad` 后实测均为正方向（见 `so101_calibration.json` 的 moves）。
- 夹爪：命令 0.0 rad → 实测 `-0.00035`；命令 1.0 rad → 实测 `0.99920`，开闭可观察。

## 真实标定（Isaac Lab / PhysX）

- 6 关节、限位弧度、home 误差 ≤ 0.1、每个关节都产生 > 0.005 rad 位移。
- 夹爪行程 0.9996 rad。
- 静止保持 1.5 s 最大漂移 `0.0349 rad`（重力下垂，仍低于 0.05 阈值）。
- 关节位置全部有限；EE body `moving_jaw_so101_v1`，home 后世界位置约 `(0.039, -0.306, 0.270)`。
- 运行 `so101_entry.py` 总耗时约 15.1 s（含 Kit 启动外的标定）。

## 物体/接触场景

- 桌面 z=0，红块 0.03 m 立方，目标盒 0.10×0.10×0.04 m。
- 红块从 `(0.18, 0, 0.03)` 落稳到 `(0.1800, -0.0000, 0.0150)`，未穿透桌面。
- 目标盒存在且为运动学静态体；接触与尺寸配置在 PhysX 中有效。

## 交付物

- `embodiments\so_arm101\robot_spec.json`（6 DOF、single gripper、skills）
- `embodiments\so_arm101\joint_mapping.json`（逻辑名/索引/限位/方向/夹爪标定）
- `embodiments\so_arm101\asset_manifest.json`（6 文件 SHA-256、license）
- `embodiments\so_arm101\scene.json`（桌高、工作空间、物体、成功区）
- `python\robotics_backends\isaaclab_physx\so101_scene.py`、`so101_entry.py`、`so101_objects_entry.py`
- `scripts\smoke_so101.ps1`、`scripts\build_so101_specs.py`、`scripts\inspect_so101_usd.py`

## 限制

- 夹爪 `max_width_m=0.04` 是几何估计，T06 需用真实抓取标定开口宽度。
- 关节增益为项目自有 `ImplicitActuatorCfg`（stiffness 20 / damping 2），不是 USD drive 原值。
- 目标盒目前是运动学静态体；T06 的放置验证要使用物理容差与保持时间。
