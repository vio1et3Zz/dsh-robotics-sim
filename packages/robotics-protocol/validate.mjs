/**
 * Validate the shared protocol fixtures against the generated JSON Schema plus
 * the semantic rules Pydantic enforces in model validators.
 *
 * JSON Schema cannot express cross-field constraints (joint array length vs
 * dof, quaternion norm, DAG cycles, conditional predicate fields).  This file
 * mirrors those rules so the Python and TypeScript boundaries accept and
 * reject the same fixtures.  Plan-hash equality stays Python-only because a
 * JSON round-trip through JavaScript loses Python float formatting.
 */
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

import Ajv2020 from "ajv/dist/2020.js";

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, "../..");
const bundlePath = path.join(root, "schemas", "v0.1", "protocol.bundle.json");
const casesPath = path.join(root, "tests", "fixtures", "protocol", "cases.json");

const bundle = JSON.parse(readFileSync(bundlePath, "utf8"));
const cases = JSON.parse(readFileSync(casesPath, "utf8"));

const ajv = new Ajv2020({
  strict: false,
  allErrors: true,
  validateFormats: false,
  logger: false,
});
ajv.addSchema(bundle);

const validators = new Map();
for (const name of Object.keys(bundle.$defs)) {
  validators.set(name, ajv.compile({ $ref: `${bundle.$id}#/$defs/${name}` }));
}

function findNonFinite(value, location = "$") {
  if (typeof value === "number") {
    return Number.isFinite(value) ? null : `${location}: non-finite number`;
  }
  if (Array.isArray(value)) {
    for (let index = 0; index < value.length; index += 1) {
      const found = findNonFinite(value[index], `${location}[${index}]`);
      if (found) return found;
    }
    return null;
  }
  if (value && typeof value === "object") {
    for (const [key, child] of Object.entries(value)) {
      const found = findNonFinite(child, `${location}.${key}`);
      if (found) return found;
    }
  }
  return null;
}

const RETRYABLE = new Set(["STALE_WORLD_STATE", "CONTROL_BUSY", "WORKER_LOST", "TIMEOUT"]);

function checkQuat(quat, label, errors) {
  if (!Array.isArray(quat) || quat.length !== 4) return;
  const norm = Math.hypot(...quat);
  if (Math.abs(norm - 1) > 1e-3) {
    errors.push(`${label}: quaternion must be normalized, got norm ${norm}`);
  }
}

function checkWorldState(state, errors) {
  const robotIds = new Set();
  for (const robot of state.robots ?? []) {
    if (robot.joint_positions.length !== robot.dof) {
      errors.push(`robot ${robot.robot_id}: joint_positions length != dof`);
    }
    if (robot.joint_velocities.length !== robot.dof) {
      errors.push(`robot ${robot.robot_id}: joint_velocities length != dof`);
    }
    checkQuat(robot.ee_pose?.orientation_xyzw, `robot ${robot.robot_id}`, errors);
    if (robotIds.has(robot.robot_id)) errors.push(`duplicate robot id ${robot.robot_id}`);
    robotIds.add(robot.robot_id);
  }
  const objectIds = new Set();
  for (const object of state.objects ?? []) {
    if (object.size_m.some((extent) => extent <= 0)) {
      errors.push(`object ${object.object_id}: size_m extents must be positive`);
    }
    checkQuat(object.pose?.orientation_xyzw, `object ${object.object_id}`, errors);
    if (objectIds.has(object.object_id)) errors.push(`duplicate object id ${object.object_id}`);
    if (object.grasped_by && !robotIds.has(object.grasped_by)) {
      errors.push(`object ${object.object_id}: grasped_by is not a declared robot`);
    }
    objectIds.add(object.object_id);
  }
  const skills = new Set();
  for (const status of state.skills ?? []) {
    if (skills.has(status.skill)) errors.push(`duplicate skill status ${status.skill}`);
    skills.add(status.skill);
  }
}

function checkRobotSpec(robot, errors) {
  if (robot.dof !== robot.joints.length) {
    errors.push(`dof ${robot.dof} != joints ${robot.joints.length}`);
  }
  const names = new Set();
  const logical = new Set();
  for (const joint of robot.joints) {
    if (names.has(joint.name)) errors.push(`duplicate joint name ${joint.name}`);
    if (logical.has(joint.logical_name)) errors.push(`duplicate logical joint ${joint.logical_name}`);
    names.add(joint.name);
    logical.add(joint.logical_name);
    if (!(joint.lower <= joint.home && joint.home <= joint.upper)) {
      errors.push(`joint ${joint.name}: home outside limits`);
    }
    if (!(joint.lower < joint.upper)) errors.push(`joint ${joint.name}: lower >= upper`);
    if (joint.type === "revolute" && joint.unit !== "rad") {
      errors.push(`joint ${joint.name}: revolute requires rad`);
    }
    if (joint.type === "prismatic" && joint.unit !== "m") {
      errors.push(`joint ${joint.name}: prismatic requires m`);
    }
  }
  if (!names.has(robot.end_effector.parent_joint)) {
    errors.push("end_effector.parent_joint is not a declared joint");
  }
  for (const name of robot.gripper.joint_names ?? []) {
    if (!names.has(name)) errors.push(`gripper joint ${name} is not declared`);
  }
  if (new Set(robot.skills).size !== robot.skills.length) errors.push("skills must be unique");
}

function checkJointSpec(joint, errors) {
  if (!(joint.lower <= joint.home && joint.home <= joint.upper)) errors.push("home outside limits");
  if (!(joint.lower < joint.upper)) errors.push("lower >= upper");
  if (joint.type === "revolute" && joint.unit !== "rad") errors.push("revolute requires rad");
  if (joint.type === "prismatic" && joint.unit !== "m") errors.push("prismatic requires m");
}

function checkRobotState(robot, errors) {
  if (robot.joint_positions.length !== robot.dof) errors.push("joint_positions length != dof");
  if (robot.joint_velocities.length !== robot.dof) errors.push("joint_velocities length != dof");
  checkQuat(robot.ee_pose?.orientation_xyzw, "ee_pose", errors);
}

function checkObjectState(object, errors) {
  if (object.size_m.some((extent) => extent <= 0)) errors.push("size_m extents must be positive");
  checkQuat(object.pose?.orientation_xyzw, "pose", errors);
}

function checkRegion(region, label, errors) {
  for (const [axis, low, high] of [
    ["x", region.min[0], region.max[0]],
    ["y", region.min[1], region.max[1]],
    ["z", region.min[2], region.max[2]],
  ]) {
    if (!(low < high)) errors.push(`${label}: min >= max on ${axis}`);
  }
}

function checkTaskSpec(task, errors) {
  const ids = new Set();
  for (const predicate of task.success_predicates) {
    if (ids.has(predicate.predicate_id)) errors.push(`duplicate predicate ${predicate.predicate_id}`);
    ids.add(predicate.predicate_id);
    if (predicate.kind === "object_in_region" && (!predicate.object_id || !predicate.region)) {
      errors.push("object_in_region requires object_id and region");
    }
    if (["object_height", "object_pose"].includes(predicate.kind) && !predicate.object_id) {
      errors.push(`${predicate.kind} requires object_id`);
    }
    if (predicate.kind === "custom" && Object.keys(predicate.params ?? {}).length === 0) {
      errors.push("custom predicate requires params");
    }
    if (predicate.region) checkRegion(predicate.region, predicate.predicate_id, errors);
  }
}

function checkTaskPlan(plan, errors) {
  const byId = new Map();
  for (const step of plan.steps) {
    if (byId.has(step.step_id)) errors.push(`duplicate step id ${step.step_id}`);
    byId.set(step.step_id, step);
  }
  for (const step of plan.steps) {
    for (const dependency of step.depends_on) {
      if (!byId.has(dependency)) errors.push(`step ${step.step_id} depends on unknown ${dependency}`);
    }
  }
  const visiting = new Set();
  const visited = new Set();
  const visit = (id) => {
    if (visiting.has(id)) {
      errors.push(`dependency cycle at ${id}`);
      return;
    }
    if (visited.has(id)) return;
    visiting.add(id);
    for (const dependency of byId.get(id)?.depends_on ?? []) visit(dependency);
    visiting.delete(id);
    visited.add(id);
  };
  for (const step of plan.steps) visit(step.step_id);
}

function checkCapabilities(capabilities, errors) {
  if (capabilities.verified && !capabilities.installed) errors.push("verified implies installed");
  if (capabilities.verified && (capabilities.evidence ?? []).length === 0) {
    errors.push("verified backend requires evidence");
  }
  if (capabilities.backend_kind === "mock" && capabilities.verified) {
    errors.push("mock backend cannot be verified");
  }
  for (const combination of capabilities.combinations ?? []) {
    if (combination.backend_kind !== capabilities.backend_kind) {
      errors.push("combination backend_kind mismatch");
    }
    if (combination.status === "verified") {
      if ((combination.evidence ?? []).length === 0) errors.push("verified combination needs evidence");
      if (!combination.checked_at) errors.push("verified combination needs checked_at");
    }
    if (["failed", "unavailable"].includes(combination.status)) {
      if ((combination.evidence ?? []).length === 0 && !combination.load_error) {
        errors.push("failed combination needs evidence or load_error");
      }
    }
  }
}

function checkSimulationSpec(spec, errors) {
  const expected = { isaaclab_physx: "physx", isaaclab_newton: "newton" }[spec.backend_kind];
  if (expected && spec.physics.engine !== expected) {
    errors.push(`${spec.backend_kind} requires physics engine ${expected}`);
  }
  if ((spec.renderer.cameras ?? []).length > 0 && !spec.renderer.enabled) {
    errors.push("cameras require the renderer");
  }
}

function checkSkillInvocation(invocation, errors) {
  if (invocation.expected_duration_s > invocation.timeout_s) {
    errors.push("expected_duration_s exceeds timeout_s");
  }
}

function checkSkillSpec(skill, errors) {
  const type = skill.parameters_schema?.type;
  if (type !== undefined && type !== "object") errors.push("parameters_schema must describe an object");
}

function checkJobEvent(event, errors) {
  if (["skill_started", "skill_finished", "verifying"].includes(event.type) && !event.skill) {
    errors.push(`${event.type} requires a skill`);
  }
  if (event.type === "failed" && !event.data?.error_code) {
    errors.push("failed event requires data.error_code");
  }
}

function checkErrorInfo(error, errors) {
  const retryable = RETRYABLE.has(error.code);
  if (error.retryable !== retryable) errors.push(`retryable must be ${retryable} for ${error.code}`);
}

function checkRuntimeManifest(manifest, errors) {
  if (manifest.source.dirty && !manifest.source.diff_sha256) {
    errors.push("dirty source requires diff_sha256");
  }
  for (const [name, version] of Object.entries(manifest.packages ?? {})) {
    if (!name || !version) errors.push("package name/version must be non-empty");
  }
}

const semanticChecks = {
  WorldState: checkWorldState,
  RobotSpec: checkRobotSpec,
  JointSpec: checkJointSpec,
  RobotState: checkRobotState,
  ObjectState: checkObjectState,
  TaskSpec: checkTaskSpec,
  TaskPlan: checkTaskPlan,
  BackendCapabilities: checkCapabilities,
  SimulationSpec: checkSimulationSpec,
  SkillInvocation: checkSkillInvocation,
  SkillSpec: checkSkillSpec,
  JobEvent: checkJobEvent,
  ErrorInfo: checkErrorInfo,
  RuntimeManifest: checkRuntimeManifest,
};

export function validateModel(model, payload) {
  const nonFinite = findNonFinite(payload);
  if (nonFinite) return { valid: false, errors: [nonFinite] };
  const validator = validators.get(model);
  if (!validator) return { valid: false, errors: [`unknown model: ${model}`] };
  if (!validator(payload)) return { valid: false, errors: validator.errors ?? [] };
  const check = semanticChecks[model];
  if (check) {
    const errors = [];
    check(payload, errors);
    if (errors.length > 0) return { valid: false, errors };
  }
  return { valid: true, errors: [] };
}

function main() {
  let failures = 0;
  for (const testCase of cases.cases) {
    const result = validateModel(testCase.model, testCase.payload);
    if (result.valid !== testCase.valid) {
      failures += 1;
      console.error(`FAIL ${testCase.name} (${testCase.model}): expected valid=${testCase.valid}, got ${result.valid}`);
      if (result.errors.length > 0) {
        console.error(`  ${JSON.stringify(result.errors.slice(0, 3))}`);
      }
    }
  }
  console.log(`schema cases: ${cases.cases.length}, failures: ${failures}`);
  if (failures > 0) process.exit(1);
}

main();
