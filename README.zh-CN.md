# DSH 机器人仿真

[English documentation](README.md)

本项目是在 Isaac Lab 中研究自然语言驱动桌面操作的实验原型。DeepSeek Harness（DSH）智能体根据场景观测构建任务计划。网关依据当前仿真状态校验计划，PhysX 仿真进程执行技能并测量物理结果。实验以 SO-ARM101 为主，Franka 场景用于接口开发。

![SO-ARM101 仿真控制台，显示彩色相机画面、机器人状态、任务与事件](artifacts/gui_2026-11-07/console.png)

*SO-ARM101 仿真运行时的网关控制台。虚拟前视相机也提供[米制深度图](artifacts/gui_2026-11-07/console-depth.png)。*

## 方法

| 组件 | 职责 |
| --- | --- |
| [DSH 插件](packages/dsh-plugin) | 提供场景查询、计划校验、执行、取消和取帧工具。 |
| [网关](python/robotics_gateway) | 校验结构化请求，将计划与场景快照关联，并将任务状态和事件写入 SQLite。 |
| [Isaac Lab 仿真进程](python/robotics_backends/isaaclab_physx) | 推进 PhysX 仿真、执行技能、采集彩色与深度帧，并从仿真状态测量任务结果。 |

抓取技能沿 x 轴每隔 1 cm 设置一个预先求解的抓取变体，共六个。机械臂通过关节位置控制完成接近、抓取、抬升、转移和释放。评估程序根据物体终态判断放置是否达到目标区域。技能运行时，控制循环以 2 Hz 发布状态，并以 1 Hz 缓存相机帧。

## 完整任务示例

**任务指令。**「抓起红色积木，放到目标盒上。」

1. 用随机种子 7 启动 `profile_so101_physx`，通过 `robotics_create_task` 创建任务，再用 `robotics_inspect_scene` 查询场景。红色积木初始位置约为 `(0.180, 0, 0.075) m`。
2. 用 `robotics_validate_plan` 按状态版本 0 校验计划。为测试恢复能力，用开发技能 `move_object` 沿 x 轴移动积木约 4 mm，状态版本更新为 1。
3. 用 `robotics_execute_task` 提交原计划。网关在执行动作前返回 `STALE_WORLD_STATE`。
4. 按状态版本 1 校验并提交新计划。任务在 13.79 秒内完成。再次查询场景，积木终态为 `(0.2326, 0.0586, 0.0550) m`，处于目标区域，随后关闭仿真。

以上步骤根据实际工具调用记录整理。 [执行记录](artifacts/e_agent_analysis_2026-11-07/recovery.json)

## 实验结果

| 实验 | 结果 | 记录 |
| --- | --- | --- |
| SO-ARM101 固定场景抓取放置 | 首轮 1/10。修正场景重置后达到 10/10，两轮采用相同的验收阈值。 | [实验协议](docs/execution/eval_manifest.json)、[结果](artifacts/t09_2026-11-07_1435/summary.md) |
| 故障恢复与取消 | 两类注入故障均在重规划后恢复。记录中的一次抓取取消由仿真进程在 217 ms 内确认。 | [恢复与生命周期测试](artifacts/t09_2026-11-07_1435/summary.md) |
| 相机输出 | 640 × 480 RGB，以及以米为单位的相机轴向深度。网关提供帧数据与元数据。 | [相机实验](artifacts/t08_2026-11-07_1410/summary.md) |

实验在单台 Windows 工作站上完成。规划与结果判定使用仿真真值，彩色图像与深度用于观测和记录。当前任务覆盖预先求解的抓取变体与固定放置位置。随机场景泛化、真机迁移和学习式视觉控制超出本次评估范围。

## 复现

已验证环境为 Windows 11、RTX 4060 Laptop GPU（8 GiB）、Isaac Sim `6.0.1.0` 和带本地修改的 Isaac Lab `v3.0.0-beta2.patch1`（提交 `ffff603e`）。仿真进程使用 Python 3.12.10 与 PyTorch `2.11.0+cu128`。

在 Python 环境中安装网关依赖，启动时指定 Isaac Lab 的解释器。按目录约定在 `assets/isaac6/` 下准备 Franka、SO-ARM101 和材质 USD 资产。仓库提供代码与验收材料，资产来源和使用条款见[第三方说明](THIRD_PARTY_NOTICES.md)。

```text
assets/isaac6/FrankaPanda/franka.usd
assets/isaac6/RobotStudio/so101_new_calib/so101_new_calib.usd
assets/isaac6/Materials/...
```

```powershell
python -m pip install -r requirements.txt
$env:PYTHONPATH = (Resolve-Path .\python).Path
$env:DSH_ROBOTICS_WARP_CACHE = 'C:\path\to\writable_ascii_cache'
python -m robotics_gateway --host 127.0.0.1 --port 8714 `
  --db artifacts\gateway.db --worker isaac --device cuda:0 `
  --python 'C:\path\to\env_isaaclab\Scripts\python.exe'
```

将示例中的两个路径替换为本机路径。Warp 缓存使用短、可写的纯 ASCII 路径。首次启动相机可能需要编译内核与着色器。启动后打开[本地控制台](http://127.0.0.1:8714/ui)。网关接口无认证，运行时绑定到回环地址。

接入 DSH 时，把 `packages/dsh-plugin` 作为本地包 `@local/dsh-robotics` 链接到当前配置，再通过插件管理器启用 `include:dsh-robotics`。插件默认连接 `http://127.0.0.1:8714`，使用其他地址时设置 `DSH_ROBOTICS_GATEWAY`。 [配置补丁](packages/dsh-plugin/cordis.patch.yml)

```powershell
python -m unittest discover -s tests/unit -v
python -m unittest tests.integration.test_gateway_http -v
```

[评估脚本](scripts/agent_eval.py)读取 [JSON 任务矩阵](configs/agent_eval_tasks.json)，并输出逐轮 JSONL 轨迹。实现细节、策略接入和故障分析见[开发记录](docs/execution/PROGRESS.md)。

## 许可证

项目代码采用 [MIT 许可证](LICENSE)。外部资产与依赖见[第三方说明](THIRD_PARTY_NOTICES.md)。
