# DSH Robotics 兼容性基线（T00）

- 日期：2026-11-07（Asia/Hong_Kong）
- 范围：本机 DSH/Cordis 宿主接口、Isaac Lab / Isaac Sim 软件基线、插件可用的工具与 Job 契约。
- 结论：接口足够实现插件；工具注册、参数 schema、执行取消信号已有本机来源。`ctx.jobs` 是抽象服务，**不能**直接假定 `ctx.jobs.start` 可在任意插件中调用；T04 需实测后再决定宿主 Job 映射。

## 1. 版本矩阵

| 组件 | 实测值 | 来源 |
|---|---|---|
| DSH desktop runtime | `0.2.2-rc.1` | `<dsh-app.asar>\dsh\package.json:3` |
| Cordis | `4.1.0` | 同上，`:7` |
| `@deepseek-ai/dsh-tools` | `0.2.2-rc.1` | 同上，`:311` |
| DSH profile | `desktop` | `$env:DSH_PROFILE` |
| profile 目录 | `%USERPROFILE%\.dsh\profiles\desktop` | `$env:DSH_PROFILE_DIR` |
| profile bundles | `@deepseek-ai/dsh-base`, `@deepseek-ai/dsh-web-app`, `dsh-opencode-go`, `dsh-agent-plugins-market` | profile `package.json` |
| Isaac Lab 源码 | `v3.0.0-beta2.patch1-dirty`，`ffff603eafc6b74264a5261cc0183d6a65390d78` | `git describe` / `rev-parse` |
| `isaaclab` 发行包 | `6.1.14`（editable，源码安装） | 包元数据 |
| Isaac Sim 包 | `6.0.1.0` | 包元数据 |
| Isaac Sim 构建 | `6.0.1-rc.7+release.42383.32955d8d.gl` | `isaacsim\VERSION` |
| `isaaclab-physx` | `1.1.3` | 包元数据 |
| Python / Torch | `3.12.10` / `2.11.0+cu128` | 包元数据 |
| Node / pnpm | `24.15.0` / 未安装 | `node --version` |
| PowerShell | Windows PowerShell `5.1.26100.9168` | `$PSVersionTable` |
| GPU | RTX 4060 Laptop，8188 MiB，驱动 616.92 | `nvidia-smi` |

## 2. 为什么 "Isaac Lab 3.0.0" 与 "isaaclab 6.1.14" 不是一回事

- `v3.0.0-beta2.patch1` 是 **Isaac Lab 源码的发布标签**，描述产品/仓库版本。
- `isaaclab 6.1.14` 是这套源码经打包后生成的 **Python 发行版本号**，由 `importlib.metadata` 读取；它不等于 "Isaac Lab 6.1"，也不能替代 Git 标签。
- Isaac Sim `6.0.1.0` 是安装包版本；真正排错要记录构建字符串 `6.0.1-rc.7+release.42383...`。
- 仓库 `VERSION` 文件只有 `3.0.0`，不足以识别 beta 与本地补丁，因此基线必须同时记录标签、提交、dirty 状态和 diff。

## 3. DSH 宿主接口（本机实测）

查询方式：`cordis_inspect_query`（platform=`host`，provider=`Service`），查询时间 2026-11-07；并读取已安装的 `dsh-tool-todo` 构建产物 `<dsh-app.asar>\dsh\node_modules\@deepseek-ai\dsh-tool-todo\lib\index.js`。

### 3.1 工具注册

```ts
register(definition: ToolDefinition, options?: ToolRegistrationOptions): () => void
```

- 在**调用方 Agent 作用域**注册；scoped 注册会遮蔽 global，同名重复与保留名 `run_code` 会失败。
- 返回精确的 disposer；插件卸载时由 Cordis 回收其注册。
- `ToolRegistrationOptions = { when?: (context: ToolActivationContext) => boolean | Promise<boolean> }`，用于按步激活。

`ToolDefinition` 的关键字段（本机契约）：

```ts
interface ToolDefinition extends ToolSchema {
  output: {
    schema: JsonSchemaNode
    render(args: unknown, value: JsonValue): ContentBlock[]
    presentationMeta?(args: unknown, value: JsonValue): JsonValue
  }
  execute(args: unknown, exec: ToolRunContext): Promise<unknown>
  presentCall?(args: unknown): ToolCallView | undefined
  presentResult?(args: unknown, result: ToolResult): ToolResultView | undefined
  timeoutMs?: number
  isConcurrencySafe?(args: unknown): boolean
}
```

执行上下文 `ToolRunContext` 继承 `ToolExecution`，可用字段至少包括：

```ts
readonly callId: ToolCallId
readonly name: string
readonly arguments: unknown
readonly agent?: Agent
readonly signal: AbortSignal   // 宿主取消信号，工具必须传递/检查
deferContext(context): void
concludeTurn(): void
```

参数 schema 约定（以本机 shipped tool 为准，不是教科书 JSON Schema）：

```js
parameters: {
  todos: {
    type: "array",
    required: true,                 // required 写在字段上
    description: "...",
    items: {
      type: "object",
      additionalProperties: false,
      properties: {
        content: { type: "string", required: true },
        status: { type: "string", required: true, enum: ["pending", "in_progress", "completed"] },
      },
    },
  },
}
```

即：字段级 `required: true`、`additionalProperties: false`、`enum` 放在字段 schema 内。不要照搬顶层 `required: [...]` 的写法。

### 3.2 插件模块形态

已安装工具插件的实际导出（`dsh-tool-todo\lib\index.js:11-20, 79-97, 198`）：

```js
import { defineTool } from "@deepseek-ai/dsh-tools";

const name = "tool-todo";
const inject = ["tools", "sessionProjections"];
const Config = z.object({ /* ... */ });
async function apply(ctx, config) {
  ctx.tools.register(defineTool({ /* ToolDefinition */ }));
}
export { Config, apply, inject, name };
```

- `name` / `inject` / `apply` / `Config` 为 Loader 约定；`inject` 声明所需服务。
- `defineTool(...)` 是类型与形状包装，字段仍以 `ToolDefinition` 为准。
- 本项目插件按 bundle 安装，安装动作由 `plugin_manager` 执行；本机没有 `pnpm`，不要手写 profile 的 `package.json` / `cordis.patch.yml`。

### 3.3 异步 Job 与展示

`ctx.jobs` 是**抽象服务**（本机契约）：

```ts
start(spec: JobSpec): JobId
list(caller?: SessionId): JobView[]
get(id, caller?): JobView
read(id, caller?): JobRead
readAt(id, from, caller?): JobOutputRead
kill(id, caller?, reason?): 'requested' | 'already-finished'
wait(id, timeoutMs, caller?, signal?): Promise<JobView>
remove(id, caller?): void
attachController(name: string): () => void
```

关键限制（必须遵守）：

- `start` 在**没有已附加 controller 服务该 owner** 时拒绝启动；实现方是 `dsh-jobs-local` 之类的插件，不是每个插件都自带。
- `JobKindMap` 当前只声明 `bash: 'bash'`；类型上 `JobSpec.kind` 只能取该值。机器人 Job 是否复用 `bash`、扩展 kind，或在 T04 先不使用 `ctx.jobs`，必须实测后决定。
- 取消语义是"请求取消"，Producer 必须自己响应 `JobHooks.cancel` 并结束 `done`；`kill` 返回不等于已停止。
- 本项目以 **Python Gateway 的 Job 为权威**；DSH Job 只做展示/取消映射，二者共享关联 ID（`docs/DSH_EXECUTION_PLAN.md` T04）。

### 3.4 相关事件

- `tools/change`：工具注册/注销或限制变化。
- `tools/pre-execute` / `tools/post-execute` / `tools/execute`：审批、超时、重试、指标。
- `tools/result`：最终结果的只读观察。

## 4. Isaac 侧接口锚点（摘要）

完整签名与行号见 `ISAAC_INTERFACE_MAP.md`；T02 直接使用这些锚点：

- `AppLauncher`（`LAB\app\app_launcher.py:229`）先创建，再导入依赖 Kit 的模块。
- `SimulationContext.reset/step/stop`（`LAB\sim\simulation_context.py:714/731/861`）。
- `InteractiveScene.write_data_to_sim/update`（`LAB\scene\interactive_scene.py:612/626`）。
- `Articulation.find_joints`（`base_articulation.py:278`）与 `set_joint_position_target_index`（`isaaclab_physx\...\articulation.py:2412`）。
- 状态是 `ProxyArray`，快照必须 `.torch` 后复制，不能把 GPU 活视图传 JSON。
- 本机四元数统一 `xyzw`（root_quat_w 与 CameraCfg.OffsetCfg.rot 均已核查），协议也采用 xyzw。
- Python Server（`127.0.0.1:8226`）是同步 TCP + 半关闭，watchdog 不能中断同步代码；只用于 OP02 的受控 dispatcher，不作为主线动作 Job。

## 5. 本机实施约束

1. **工作区路径含中文**。本地补丁 `play_rsl_rl.py` 已处理 TorchScript 在非 ASCII 路径下无法保存的问题，导出改走 `%LOCALAPPDATA%\IsaacLab\exported\...`。本项目所有 TorchScript/ONNX 导出、需要交给原生库的临时文件都走 ASCII 路径；普通 JSON/日志可留在工作区。
2. **PowerShell 5.1，执行策略 `RemoteSigned`**。脚本不要用 `??`、三元表达式、`-Parallel` 等 PS 7 语法；`doctor.ps1` 已按 5.1 编写。当前会话直接 `& script.ps1` 会被拒绝，需用 `powershell.exe -NoProfile -ExecutionPolicy Bypass -File <script>` 启动（或由用户调整策略）。
3. **无 `pnpm`**，Node 仅 `24.15.0`。插件安装走 `plugin_manager`；Python 依赖用各环境自己的 pip。
4. **8 GiB 显存**。首版一环境、低分辨率单相机；语言模型走远程，不在本机同时加载大 VLA。
5. **资产根是远程 URL**。`apps/isaaclab.python.kit` 的 `asset_root.default` 指向 `https://.../Assets/Isaac/6.0`；本机 `<isaac-root>\assets`、`%USERPROFILE%\AppData\Local\ov\pkg` 均不存在，深度 4 扫描未发现 `so101` 文件。T05 必须实际验证下载、引用依赖和许可，不能假设可用。
6. **历史盘符遗留**：`<isaac-root>\IsaacLab\_` 是未跟踪的历史崩溃栈（内容引用已废弃盘符的 isaac 路径），仅作背景，不作为路径依赖；不修改上游工作区。`doctor.ps1` 会扫描本项目 `configs/scripts/docs` 中的历史盘符引用。
7. **DSH profile 只读**：不写 `%USERPROFILE%\.dsh\profiles\desktop` 下的 `package.json` / `cordis.patch.yml`；安装由 `plugin_manager` 完成。
8. **Worker 运行约定**：工作目录为 `dsh-robotics`；启动复用 `<isaac-root>\isaaclab.cmd` 的环境逻辑（激活 `env_isaaclab`、`PYTHONPATH` 加 `grasp_env_generator\source` 与 `IsaacLab\source\isaaclab`、清空 `ISAAC_PATH`/`EXP_PATH`/`CARB_APP_PATH`/`PYTHONHOME`）。日志写 `artifacts\<run_id>\`；需要 TorchScript/ONNX 或原生库读取的产物走 ASCII 路径（见第 1 条）。

## 6. 资产获取策略（T02 实测）

- Isaac 6.0 资产根下，`FRANKA_PANDA_CFG` 默认引用的 `.../Isaac/IsaacLab/Robots/FrankaEmika/panda_instanceable.usd` 返回 **404**；同根下 `.../Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` 返回 200。
- 本机 `omni.client.stat` 无法打开远程 HTTPS 资产（即使 Python `urllib` 能取到）；直接给远程 URL 时 Kit 报 `USD file not found`。
- 因此 T02 把 `FrankaPanda` 整棵资产树镜像到 `assets/isaac6/FrankaPanda/`（约 39 MB，含 LICENSE），Worker 用本地路径加载。`franka.usd` 的 SHA-256 为 `aa480a0c26cfacedd223259bd82b0825a44f857a27769db8be1d34c3f82c6d0f`，已写入 `runtime_manifest.json` 的 `assets`。
- 默认 `GroundPlaneCfg` 也引用远程 `default_environment.usd`，同样不可用；T02 改成程序化 `CuboidCfg` 地面，不依赖远程环境资产。
- T05 的 SO-ARM101 必须走同一策略：先确认 S3 键存在、镜像到项目 `assets/`、核对 LICENSE 与哈希，再加载；不能把“安装包里列出了资产”当成可用。
- 失败证据保留：`artifacts\t02_2026-11-07_0050`（默认 Panda 资产 404）、`0100`（远程 franka.usd 经 `omni.client` 打不开）、`0110`（远程地面 404）；`0120` 为首次全通过（无完整控制台日志），`0150` 为最终完整证据。

## 7. T00 验收对照

| 验收项 | 证据 | 状态 |
|---|---|---|
| doctor 区分"包已安装 / 服务存活 / 任务已验证" | `scripts\doctor.ps1` 输出三段式检查；`artifacts\t00_2026-11-07_0015\doctor.json` | 待运行 |
| 解释 Lab 产品版本与包版本不同 | 本文第 2 节 | 完成 |
| DSH 工具注册 API 有本机来源 | 本文 3.1–3.2，来自 live Inspect 与 `dsh-tool-todo` 构建产物 | 完成 |
| 不依赖旧盘符路径 | `doctor.ps1` 的 `paths.legacyDrives` 检查；本文第 5.6 节 | 待运行 |

## 8. 未决项

- `ctx.jobs.start` 在本 profile 是否可用（需 T04 实测 `attachController` 与 kind 约束）。
- Gateway 独立环境与端口（默认 8710）需 T03 固定并写入 `configs\local.json`。
- SO-ARM101 USD 的下载路径、缓存位置、许可与 Newton 兼容性（T05/OP01）。
- GUI timeline 暂停时 `sim.step` 的阻塞行为（T03 专项测试）。
- 真实 DSH 会话里工具取消信号到 Python Job 的端到端延迟（T04 测量）。
