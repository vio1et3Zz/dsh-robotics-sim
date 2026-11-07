# T06 — 受约束技能与独立物理验证（passed）

- 时间：2026-11-07 13:30 (+08:00)
- 工程：`<repo>`
- 场景：`profile_so101_physx` 同一 SO-ARM101 物体场景（平台 + 红块 + 目标盒）
- 结论：**真实 PhysX 下可重复 pick/place 通过**；三次独立启动的结果逐位相同（本机 PhysX 确定性）。

## 1. 夹爪几何（从碰撞网格测得）

AABB 摘要会误导，实际夹爪是两个**上下叠放的夹条**，开口朝 palm 局部 -Z：

| 量 | palm 局部坐标 |
|---|---|
| 固定夹条内表面 | x = -0.0079 m（在块高深度处） |
| 活动夹条内表面（闭合） | x = +0.0099 m |
| 开口中心（块中心） | (0.0071, 0, -0.085) m |
| 夹爪铰链 | (0.0202, 0.0188, -0.0234) m，轴为 palm -Y |
| 碰撞几何 | 与视觉网格一致（`collision_prototypes` 对比，点数/包围盒相同） |

证据：`gripper_points.json`（palm 143,628 点、jaw 84,810 点，body 局部坐标）、
`gripper_views.png`（三视图）、`so101_collisions_raw.json`、`gripper_debug.jsonl`。

早期失败原因：把 palm 中心/body 原点当作夹持点，夹条从未真正跨过物块。

## 2. 可达性（离线 USD 链 + scipy）

- 由 USD 关节 rest 变换建立 5 轴串链；rest palm 位置误差 **1.54e-05 m**。
- 顶抓（palm 局部 Z = 世界 Z，口朝下）在 x=0.16–0.26 均有**精确解**（0.00 mm / 0.00°）。
- 侧抓 +X 在 x=0.16 可解但物理 pose-IK 收敛差；-X 方向姿态不可达；顶抓被选为 T06 主轨迹。

证据：`kinematics_sweep2.json`、`pick_waypoints2.json`、`seed_top_018.json`、`so101_kinematics.py`。

## 3. 固定轨迹（顶抓）

| 路点 | 关节角（度，五个臂关节） | 嘴部目标 |
|---|---|---|
| approach | -94.01, -16.81, 20.65, 86.16, 85.99 | (0.18, -0.005, 0.115) |
| grasp | -94.01, -14.69, 31.48, 73.22, 85.99 | (0.18, -0.005, 0.085) |
| lift1 | -94.01, -16.49, 24.59, 81.90, 85.99 | (0.18, -0.005, 0.105) |
| lift2 | -94.01, -16.61, 14.09, 92.53, 85.99 | (0.18, -0.005, 0.130) |
| transfer | -108.37, 7.93, -13.92, 95.00, 71.58 | (0.22, 0.05, 0.130) |
| place | -108.37, 10.81, 5.67, 73.51, 71.63 | (0.22, 0.05, 0.075) |
| retreat | -108.37, -1.66, -13.47, 95.00, 71.44 | (0.22, 0.05, 0.160) |

关键修正：

1. **抓取 y 偏移 −5 mm**：若嘴部正好对准块中心，固定夹条内表面与块 -Y 面恰好接触（离线间隙 0.0 mm），下降会把块推走；偏移后间隙 5 mm。
2. **保持指令目标**：下降前若把臂目标重置为“当前实测位姿”（会下垂），轨迹会扰动块；保持上一路点指令后下降干净（离线逐点间隙 ≥5.4 mm）。
3. 关闭夹爪时同样保持抓取路点目标，避免臂回弹。

## 4. 真实 PhysX 结果（三次一致）

| 量 | 值（m） |
|---|---|
| 块初始 | (0.18000, 0.00000, 0.07500) |
| 抓取时嘴部 | (0.17588, -0.00530, 0.08108) |
| 抓取时块 | (0.18000, 0.00000, 0.07500)（未被扰动） |
| 闭合后块 | (0.18141, -0.00071, 0.07626) |
| lift1 块 | (0.18176, 0.00028, 0.09342) |
| lift2 块 | (0.18285, 0.00023, 0.11812)（抬升 **+43 mm**） |
| transfer 块 | (0.22645, 0.05584, 0.11501) |
| place 块 | (0.22989, 0.05712, 0.06524) |
| 释放/退让后 | (0.23019, 0.05837, 0.05500)（在目标盒顶面） |

判定：`block_untouched_on_approach`、`grasp_centered`、`lifted`、
`block_follows_mouth`、`carried_to_place`、`placed_in_target_region`、`released` 全为 true。

运行：`so101_waypoint_pick_probe.py` 三次（`waypoint_pick3/4/5.json`），结果相同；
`so101_skill_probe.py --mode success`（`skill_success3.json`）用技能类复现同样结果。

## 5. 技能封装：`SO101PickPlace`

`python\robotics_backends\isaaclab_physx\so101_pick_place.py`

- `precheck(place_xy)`：可达区域（x 0.15–0.27、y ±0.08）、物体存在、物体在工作区；失败返回
  `UNREACHABLE_TARGET` / `OBJECT_NOT_FOUND` / `OBJECT_OUT_OF_WORKSPACE`，且**不动作**。
- `execute()`：approach → grasp → close → lift1 → lift2 → transfer → place → release → retreat。
- 每个阶段独立物理判定；夹爪闭合本身从不等于抓取成功。
- 取消：`cancel_check` 在每步检查，取消时主动保持当前位姿并抛 `SkillCanceled`（与 T03 语义一致）。
- 故障注入钩子 `fault_after_grasp` 仅用于测试。

## 6. 明确失败（真实运行）

| 模式 | 注入 | 结果 | 证据 |
|---|---|---|---|
| EMPTY_GRASP | 抓取校验后把块移走，闭合夹空 | `passed=false`, `lifted=false` | `skill_empty.json` |
| GRASP_MISSED | 块移到 (0.20, 0, 0.075)，仍用固定轨迹 | `grasp_centered=false`，未抬升 | `skill_grasp_missed.json` |
| UNREACHABLE_TARGET | place=(0.30, 0.20) | 预检拒绝，关节指令与实测均未变化 | `skill_unreachable.json` |

## 7. 交付与限制

- 场景：`so101_objects_scene.py`（目标盒移到 (0.22, 0.05, 0.02)，使放置姿态可达）。
- 固定轨迹仅覆盖该场景；任意目标位置需运行离线求解器生成新路点（T07 重规划）。
- 目标盒为 kinematic 静态体，放置成功以块最终位姿判定，不是盒内传感器。
- 本机 Kit 启动偶发无输出退出（`skill_success`/`skill_success2` 复现两次），重跑成功；不作为通过依据。
- 抓取精度依赖实测嘴部坐标系；未使用 Franka 参数或偏移。

## 8. 证据文件

- `waypoint_pick3.json` / `4` / `5`：三次一致的完整 pick/place。
- `skill_success3.json`：技能类成功运行（含阶段测量）。
- `skill_empty.json`、`skill_grasp_missed.json`、`skill_unreachable.json`：三类明确失败。
- `pick_waypoints2.json`、`kinematics_sweep2.json`、`seed_top_018.json`：轨迹解与可达性。
- `gripper_points.json`、`gripper_views.png`、`descent_diag1.json`、`so101_collisions_raw.json`：夹爪几何与下降间隙。
- 日志：`waypoint_pick3/4/5.log`、`skill_*.log`、`descent_diag1.log`。
