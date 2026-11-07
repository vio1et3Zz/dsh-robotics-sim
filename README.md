# DSH Robotics Sim

[中文文档](README.zh-CN.md)

DSH Robotics Sim gives a DeepSeek Harness (DSH) agent access to a running robotics simulation. The project is designed for experiments with vision-language-action (VLA) policies. A policy produces robot actions, while the agent observes the rollout, reads the outcome, and decides whether to let the policy continue, stop it, or take over the task with a simulator skill.

The current backend is Isaac Lab with PhysX. The DSH tools access the simulator through a local gateway, which exposes scene state, camera frames, job events and execution controls. The same tool boundary can be used to add other simulators through backend adapters.

![Gateway console showing the SO-ARM101 scene, RGB camera, robot state, jobs and events](artifacts/gui_2026-11-07/console.png)

*The console displays the scene, job state and camera data exposed by the gateway. [Depth view](artifacts/gui_2026-11-07/console-depth.png)*

## From a policy rollout to agent intervention

| Step | Agent interaction |
| --- | --- |
| Observe | During a rollout, read the robot and object state with `robotics_inspect_scene`, retrieve the latest camera frame with `robotics_latest_frame`, and follow execution with `robotics_get_job_events`. |
| Interpret | Compare the policy outcome with the task goal using failure codes, scene state and recorded frames. The agent can use the same records after a rollout to diagnose a failure and propose a retry. |
| Intervene | Request cancellation with `robotics_cancel_job` if the active job should stop. Wait for the worker to acknowledge the stop before sending a replacement action. |
| Take over | Read the latest scene, validate a revised plan, and submit a recovery skill or a new rollout through the gateway. The gateway checks the plan against the current state before execution. |

The prototype provides monitoring during execution, cancellation with worker acknowledgement, and plan validation and execution. The policy adapter accepts observations and returns action chunks. A scripted policy exercises that interface today. Connecting a trained VLA model is the next integration step. [Policy adapter](python/robotics_backends/isaaclab_physx/stub_policy.py)

## Example: diagnose and retry a missed grasp

**Task.** Pick up the red block with SO-ARM101 and place it on the target box.

1. Read the rollout report and the frame recorded at `GRASP_MISSED`. The scene placed the block near `x = 0.19 m`, while the grasp request forced `grasp_x = 0.16 m`.
2. Compare the frame with the recorded parameters. The gripper closed beside the block, consistent with the 3 cm offset.
3. Retry the same scene without the forced grasp position so the worker selects the variant from the measured block position. Read the final scene to confirm placement in the target region.

This recorded example illustrates diagnosis and retry through the simulator interface. A separate tool trace records the agent revising a plan after the gateway detected a scene change. [Failure analysis and retry](artifacts/e_agent_analysis_2026-11-07/analysis.md) · [Replanning trace](artifacts/e_agent_analysis_2026-11-07/recovery.json)

The SO-ARM101 and Franka scenes were used to verify the interface on the local Isaac installation. [Interface verification notes](docs/execution/PROGRESS.md)

## Reproduction

The tested system used Windows 11, an RTX 4060 Laptop GPU with 8 GiB of memory, Isaac Sim `6.0.1.0`, and Isaac Lab `v3.0.0-beta2.patch1` at commit `ffff603e` with local modifications. The worker used Python 3.12.10 and PyTorch `2.11.0+cu128`.

Install the gateway dependencies in a Python environment and supply the Isaac Lab interpreter when starting the service. Place the Franka, SO-ARM101 and material USD assets under `assets/isaac6/` using the documented directory layout. The repository distributes code and evaluation artifacts. Asset sources and terms are listed separately. [Third-party notices](THIRD_PARTY_NOTICES.md)

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

Replace both example paths with local paths. Use a short, writable ASCII path for the Warp cache. The first camera startup may take longer while kernels and shaders compile. Open [the local console](http://127.0.0.1:8714/ui). Run the gateway on loopback because its local API has no authentication.

To connect DSH, link `packages/dsh-plugin` as the local package `@local/dsh-robotics` in the active profile and enable `include:dsh-robotics` through the plugin manager. The plugin defaults to `http://127.0.0.1:8714`. Set `DSH_ROBOTICS_GATEWAY` when using a different address. [Profile patch](packages/dsh-plugin/cordis.patch.yml)

```powershell
python -m unittest discover -s tests/unit -v
python -m unittest tests.integration.test_gateway_http -v
```

The [evaluation runner](scripts/agent_eval.py) uses [JSON task matrices](configs/agent_eval_tasks.json) and writes per-episode JSONL traces. See the [development record](docs/execution/PROGRESS.md) for implementation details, the policy adapter and the failure analysis.

## License

The project code is available under the [MIT License](LICENSE). See the [third-party notices](THIRD_PARTY_NOTICES.md) for external assets and dependencies.
