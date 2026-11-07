# Third-party notices

本仓库代码以 MIT 发布（见 `LICENSE`）。以下第三方组件不随仓库分发，或仅以依赖/引用形式使用；
使用或再分发前请以上游当前许可证与署名要求为准。

## 机器人资产（Apache-2.0，不随仓库分发）

- **Franka Emika Panda**：来自 Isaac Lab / `isaaclab_assets` 的 FrankaPanda USD，
  Apache-2.0。作者本地镜像路径：`assets/isaac6/FrankaPanda/`（原目录含 LICENSE）。
- **SO-ARM101（RobotStudio so101_new_calib）**：Apache-2.0。本地镜像路径：
  `assets/isaac6/RobotStudio/so101_new_calib/`（原目录含 LICENSE）。
- **Materials**：Isaac Lab 材质资源，Apache-2.0。本地镜像路径：`assets/isaac6/Materials/`。

## 仿真与运行时（不随仓库分发）

- **Isaac Sim / Isaac Lab**：NVIDIA 许可（见其安装目录的 LICENSE）；本仓库不包含其二进制。
- **NVIDIA Warp**：NVIDIA 许可；相机 kernel 的编译缓存由用户本机生成。
- **PyTorch**：BSD-3-Clause。

## Python 依赖（pip 安装，版本见 `requirements.txt`）

| 包 | 许可证 |
|---|---|
| FastAPI | MIT |
| Uvicorn | BSD-3-Clause |
| Pydantic | MIT |
| NumPy | BSD-3-Clause |
| Pillow | HPND |
| SciPy | BSD-3-Clause |

## Node 依赖（`packages/robotics-protocol`）

| 包 | 许可证 |
|---|---|
| Ajv | MIT |
| TypeScript | Apache-2.0 |

## 说明

- 本仓库的 `artifacts/` 只保留小型运行摘要/报告；帧、数据库、日志与 worker 输出目录已被
  `.gitignore` 排除。
- 若你要把本仓库用于公开再分发，请自行复核上述上游许可证的最新版本与署名要求。
