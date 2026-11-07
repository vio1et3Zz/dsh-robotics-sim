/**
 * DSH Robotics Host plugin.
 *
 * These tools are the Agent-facing contract; they call the local Robotics
 * Gateway over HTTP.  The Gateway (not this plugin) owns job state, cancellation
 * and Worker acknowledgement.  Tools return quickly for long operations; poll
 * `robotics_get_job_status` for progress.
 */
const BASE = process.env.DSH_ROBOTICS_GATEWAY ?? "http://127.0.0.1:8714";

const PROFILE_SPECS = {
  "profile_dev_franka_physx": {
    backend_kind: "isaaclab_physx",
    robot_id: "franka",
    scene_id: "table_top",
    physics: { engine: "physx", dt: 0.005, decimation: 4, solver_iterations: 8 },
    renderer: { enabled: false, headless: true },
    num_envs: 1,
    budget: { max_sim_time_s: 120.0, max_wall_time_s: 300.0 },
  },
  "profile_mock_franka": {
    backend_kind: "mock",
    robot_id: "franka",
    scene_id: "table_top",
    physics: { engine: "physx", dt: 0.005, decimation: 4, solver_iterations: 8 },
    renderer: { enabled: false, headless: true },
    num_envs: 1,
    budget: { max_sim_time_s: 60.0, max_wall_time_s: 120.0 },
  },
  "profile_so101_physx": {
    backend_kind: "isaaclab_physx",
    robot_id: "so101",
    scene_id: "table_pick_place",
    physics: { engine: "physx", dt: 0.005, decimation: 4, solver_iterations: 8 },
    renderer: {
      enabled: true,
      headless: true,
      backend: "isaac_rtx",
      cameras: [
        {
          name: "front",
          width: 640,
          height: 480,
          update_period_s: 0.1,
          data_types: ["rgb", "depth"],
        },
      ],
    },
    num_envs: 1,
    budget: { max_sim_time_s: 180.0, max_wall_time_s: 420.0 },
  },
};

async function call(path, { method = "GET", body, signal } = {}) {
  const response = await fetch(`${BASE}${path}`, {
    method,
    headers: body === undefined ? undefined : { "content-type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
    signal,
  });
  const text = await response.text();
  let data = {};
  if (text) {
    try {
      data = JSON.parse(text);
    } catch {
      data = { raw: text };
    }
  }
  if (!response.ok) {
    const error = data.error ?? { code: `HTTP_${response.status}`, message: text || response.statusText };
    const failure = new Error(`${error.code}: ${error.message}`);
    failure.code = error.code;
    throw failure;
  }
  return data;
}

function textResult(value) {
  return [{ type: "text", text: JSON.stringify(value, null, 2) }];
}

function defineTool({ name, description, parameters, run }) {
  return {
    name,
    description,
    parameters,
    output: {
      schema: { type: "object", additionalProperties: true },
      render: (_args, value) => textResult(value),
    },
    async execute(args, exec) {
      return run(args ?? {}, exec);
    },
  };
}

const simulationId = {
  type: "string",
  required: true,
  description: "Simulation id returned by robotics_start_simulation, e.g. sim_....",
};
const jobId = {
  type: "string",
  required: true,
  description: "Gateway job id, e.g. job_....",
};
const idempotencyKey = {
  type: "string",
  description: "Optional idempotency key. Repeating the same key returns the existing job.",
};
const optionalObject = {
  type: "object",
  additionalProperties: true,
};

export const inject = ["tools"];

const TOOLS = [
  defineTool({
    name: "robotics_list_backends",
    description:
      "List the Robotics Gateway backends, their liveness and their supported skills. Use this before starting a simulation.",
    parameters: {},
    run: ({}, exec) => call("/api/v1/backends", { signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_start_simulation",
    description:
      "Start one simulation from a fixed Worker profile and return a startup job. Poll robotics_get_job_status until it succeeds; the job then carries the simulation_id.",
    parameters: {
      profile_id: {
        type: "string",
        required: true,
        enum: Object.keys(PROFILE_SPECS),
        description: "Worker profile allowlist entry.",
      },
      seed: { type: "integer", description: "Optional seed; defaults to 1." },
      idempotency_key: idempotencyKey,
    },
    run: ({ profile_id, seed = 1, idempotency_key }, exec) => {
      const profile = PROFILE_SPECS[profile_id];
      if (!profile) throw new Error(`unknown profile_id: ${profile_id}`);
      const spec = { ...profile, profile_id, seed };
      return call("/api/v1/simulations", {
        method: "POST",
        body: { spec, idempotency_key },
        signal: exec?.signal,
      });
    },
  }),
  defineTool({
    name: "robotics_inspect_scene",
    description:
      "Return the current scene snapshot: robot joints and end-effector pose, objects, episode, state_version and the skills this backend supports.",
    parameters: { simulation_id: simulationId },
    run: ({ simulation_id }, exec) =>
      call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}/state`, { signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_get_robot_state",
    description: "Return only the robot state (joint positions/velocities, end-effector pose, gripper).",
    parameters: { simulation_id: simulationId },
    run: async ({ simulation_id }, exec) => {
      const data = await call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}/state`, {
        signal: exec?.signal,
      });
      return {
        simulation_id: data.simulation_id,
        episode_id: data.episode_id,
        state_version: data.state_version,
        robots: data.state?.robots ?? [],
      };
    },
  }),
  defineTool({
    name: "robotics_run_skill",
    description:
      "Submit one bounded development skill (for example calibrate_joint) and return a skill job. Poll robotics_get_job_status; use robotics_cancel_job to request a Worker-acknowledged stop.",
    parameters: {
      simulation_id: simulationId,
      skill: { type: "string", required: true, description: "Skill name from robotics_list_backends." },
      parameters: optionalObject,
      target_object_id: { type: "string", description: "Optional target object id." },
      expected_duration_s: { type: "number", description: "Expected duration; defaults to 5 s." },
      timeout_s: { type: "number", description: "Hard timeout; defaults to 30 s." },
      idempotency_key: idempotencyKey,
    },
    run: ({ simulation_id, skill, parameters = {}, target_object_id, expected_duration_s = 5, timeout_s = 30, idempotency_key }, exec) =>
      call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}/skills`, {
        method: "POST",
        body: {
          invocation: {
            skill,
            parameters,
            target_object_id,
            frame: "world",
            expected_duration_s,
            timeout_s,
          },
          idempotency_key,
        },
        signal: exec?.signal,
      }),
  }),
  defineTool({
    name: "robotics_get_job_status",
    description: "Read one Gateway job: status, progress, error and event-derived evidence.",
    parameters: { job_id: jobId },
    run: ({ job_id }, exec) => call(`/api/v1/jobs/${encodeURIComponent(job_id)}`, { signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_get_job_events",
    description:
      "Read one job's event timeline (created/started/progress/succeeded/failed/canceled) with a durable sequence number. Page with after_seq; the returned next_seq is the cursor for the next call.",
    parameters: {
      job_id: jobId,
      after_seq: { type: "integer", description: "Return events with seq > after_seq; defaults to 0." },
      limit: { type: "integer", description: "Maximum events to return; defaults to 200." },
    },
    run: ({ job_id, after_seq = 0, limit = 200 }, exec) =>
      call(
        `/api/v1/jobs/${encodeURIComponent(job_id)}/events?after_seq=${encodeURIComponent(after_seq)}&limit=${encodeURIComponent(limit)}`,
        { signal: exec?.signal },
      ),
  }),
  defineTool({
    name: "robotics_cancel_job",
    description:
      "Request cancellation. The job becomes canceling immediately and canceled only after the Worker confirms it stopped; poll robotics_get_job_status.",
    parameters: { job_id: jobId },
    run: ({ job_id }, exec) =>
      call(`/api/v1/jobs/${encodeURIComponent(job_id)}/cancel`, { method: "POST", signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_reset_simulation",
    description: "Start a new episode. Rejected with CONTROL_BUSY while a job is live; cancel first.",
    parameters: { simulation_id: simulationId },
    run: ({ simulation_id }, exec) =>
      call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}/reset`, {
        method: "POST",
        signal: exec?.signal,
      }),
  }),
  defineTool({
    name: "robotics_close_simulation",
    description: "Close the simulation and release the Worker. Rejected while a job is live.",
    parameters: { simulation_id: simulationId },
    run: ({ simulation_id }, exec) =>
      call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}`, {
        method: "DELETE",
        signal: exec?.signal,
      }),
  }),
  defineTool({
    name: "robotics_capture_frame",
    description:
      "Render one front RGB/depth frame for a live simulation. Returns metadata (intrinsics, extrinsics, units, sim_step, timestamp) plus rgb/depth URLs; the frames are stored by the Gateway, not pushed over the event stream.",
    parameters: { simulation_id: simulationId },
    run: ({ simulation_id }, exec) =>
      call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}/frames`, {
        method: "POST",
        signal: exec?.signal,
      }),
  }),
  defineTool({
    name: "robotics_get_frame",
    description: "Read one stored frame's metadata, RGB/depth URLs, intrinsics and extrinsics.",
    parameters: {
      frame_id: { type: "string", required: true, description: "Frame id, e.g. frame_...." },
    },
    run: ({ frame_id }, exec) => call(`/api/v1/frames/${encodeURIComponent(frame_id)}`, { signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_latest_frame",
    description:
      "Read the newest stored frame for a simulation without rendering a new one. Use this while a skill or policy is running to watch the camera; call robotics_capture_frame when a fresh frame is required.",
    parameters: { simulation_id: simulationId },
    run: ({ simulation_id }, exec) =>
      call(`/api/v1/simulations/${encodeURIComponent(simulation_id)}/frames/latest`, { signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_create_task",
    description:
      "Create a task from a full TaskSpec object (task_id, goal, robot_id, scene_id, success_predicates, budgets). This is the prerequisite for plan validation and execution.",
    parameters: {
      spec: { type: "object", required: true, additionalProperties: true },
    },
    run: ({ spec }, exec) => call("/api/v1/tasks", { method: "POST", body: { spec }, signal: exec?.signal }),
  }),
  defineTool({
    name: "robotics_validate_plan",
    description:
      "Validate an Agent-authored TaskPlan against the task, robot skills, budget and current episode. Returns the plan_hash required by robotics_execute_task.",
    parameters: {
      task_id: { type: "string", required: true },
      plan: { type: "object", required: true, additionalProperties: true },
    },
    run: ({ task_id, plan }, exec) =>
      call(`/api/v1/tasks/${encodeURIComponent(task_id)}/plans/validate`, {
        method: "POST",
        body: { plan },
        signal: exec?.signal,
      }),
  }),
  defineTool({
    name: "robotics_execute_task",
    description:
      "Execute a previously validated plan by plan_hash and return a task job. The Gateway re-checks the episode/state_version and rejects stale plans.",
    parameters: {
      task_id: { type: "string", required: true },
      simulation_id: simulationId,
      plan_hash: { type: "string", required: true },
      idempotency_key: idempotencyKey,
    },
    run: ({ task_id, simulation_id, plan_hash, idempotency_key }, exec) =>
      call(`/api/v1/tasks/${encodeURIComponent(task_id)}/execute`, {
        method: "POST",
        body: { simulation_id, plan_hash, idempotency_key },
        signal: exec?.signal,
      }),
  }),
];

export function apply(ctx) {
  for (const tool of TOOLS) {
    ctx.effect(() => ctx.tools.register(tool));
  }
}
