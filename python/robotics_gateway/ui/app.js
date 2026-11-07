/* DSH Robotics Gateway console.
 *
 * Plain browser JavaScript: no build step, no external assets.  Every request
 * goes to the Gateway's own REST/WS API; the page is served by that Gateway, so
 * there is no CORS and no auth (loopback only).
 */
"use strict";

const $ = (id) => document.getElementById(id);
const LIVE = new Set(["pending", "running", "canceling"]);
const TERMINAL = new Set(["succeeded", "failed", "canceled"]);
const DEPTH_DEFAULT = new URLSearchParams(location.search).get("depth") === "1";

/* Keep these in sync with packages/dsh-plugin/index.js PROFILE_SPECS: the
 * Gateway resolves the profile from this body, so clients never send an
 * executable. */
const PROFILES = {
  profile_so101_physx: {
    backend_kind: "isaaclab_physx",
    robot_id: "so101",
    scene_id: "table_pick_place",
    physics: { engine: "physx", dt: 0.005, decimation: 4, solver_iterations: 8 },
    renderer: {
      enabled: true,
      headless: true,
      backend: "isaac_rtx",
      cameras: [{ name: "front", width: 640, height: 480, update_period_s: 0.1, data_types: ["rgb", "depth"] }],
    },
    num_envs: 1,
    budget: { max_sim_time_s: 180.0, max_wall_time_s: 420.0 },
  },
  profile_dev_franka_physx: {
    backend_kind: "isaaclab_physx",
    robot_id: "franka",
    scene_id: "table_top",
    physics: { engine: "physx", dt: 0.005, decimation: 4, solver_iterations: 8 },
    renderer: { enabled: false, headless: true },
    num_envs: 1,
    budget: { max_sim_time_s: 120.0, max_wall_time_s: 300.0 },
  },
  profile_mock_franka: {
    backend_kind: "mock",
    robot_id: "franka",
    scene_id: "table_top",
    physics: { engine: "physx", dt: 0.005, decimation: 4, solver_iterations: 8 },
    renderer: { enabled: false, headless: true },
    num_envs: 1,
    budget: { max_sim_time_s: 60.0, max_wall_time_s: 120.0 },
  },
};

const SKILL_TEMPLATES = {
  observe_scene: { parameters: {}, timeout_s: 30, expected_duration_s: 10 },
  hold_pose: { parameters: { duration_s: 3 }, timeout_s: 30, expected_duration_s: 10 },
  pick: { parameters: { place_x: 0.22, place_y: 0.05 }, timeout_s: 180, expected_duration_s: 90 },
  move_object: {
    parameters: { object_id: "red_block", position: [0.2, 0.0, 0.05] },
    timeout_s: 60,
    expected_duration_s: 20,
  },
};

const state = {
  sims: [],
  simId: null,
  world: null,
  frames: [],
  jobs: [],
  selectedFrame: null,
  depthMode: false,
  depthCache: new Map(),
  lastSeq: 0,
  ws: null,
  busy: new Set(),
  skillSignature: "",
};

/* ---------------------------------------------------------------- utils -- */

const short = (id, n = 14) => (id && id.length > n ? id.slice(0, n) + "…" : String(id || "—"));
const fmt = (v, d = 3) => (Number.isFinite(Number(v)) ? Number(v).toFixed(d) : "—");
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value === undefined || value === null) continue;
    if (key === "class") node.className = value;
    else if (key === "text") node.textContent = value;
    else if (key.startsWith("on")) node.addEventListener(key.slice(2), value);
    else node.setAttribute(key, value);
  }
  for (const child of [].concat(children)) {
    if (child === undefined || child === null) continue;
    node.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
  return node;
}

function timeOf(iso) {
  if (!iso) return "—";
  const date = new Date(iso);
  return Number.isNaN(date.getTime()) ? iso.slice(11, 19) : date.toLocaleTimeString("zh-CN", { hour12: false });
}

let toastTimer = null;
function toast(message, kind = "error") {
  const box = $("toast");
  box.textContent = message;
  box.className = "toast " + kind;
  box.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    box.hidden = true;
  }, 6000);
}

/* ------------------------------------------------------------------ api -- */

async function api(path, options = {}) {
  const init = { method: options.method || "GET" };
  if (options.body !== undefined) {
    init.headers = { "content-type": "application/json" };
    init.body = JSON.stringify(options.body);
  }
  let response;
  try {
    response = await fetch(path, init);
  } catch (error) {
    const failure = new Error("Gateway unreachable: " + error.message);
    failure.code = "NETWORK";
    throw failure;
  }
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
    const info = data.error || {};
    const failure = new Error(`${info.code || "HTTP_" + response.status}: ${info.message || response.statusText}`);
    failure.code = info.code;
    failure.status = response.status;
    failure.data = data;
    throw failure;
  }
  return data;
}

/* --------------------------------------------------------------- pills -- */

function setPill(id, text, tone = "") {
  const node = $(id);
  if (!node) return;
  node.textContent = text;
  node.className = "pill" + (tone ? " " + tone : "");
}

async function refreshStatus() {
  try {
    const [health, backends] = await Promise.all([api("/api/v1/health"), api("/api/v1/backends")]);
    setPill("pill-gateway", "gateway: " + health.gateway, health.gateway === "ok" ? "ok" : "err");
    setPill("pill-worker", "worker: " + health.worker, health.worker === "alive" ? "ok" : "warn");
    setPill("pill-ready", "sim ready: " + (health.simulation_ready ? "yes" : "no"), health.simulation_ready ? "ok" : "");
    const lost = Number(health.startup_lost_jobs || 0);
    setPill("pill-startup", "startup lost: " + lost, lost ? "warn" : "");
    const backend = (backends.backends || [])[0];
    if (backend) {
      const skills = backend.skills || [];
      $("backend-skills").textContent = backend.kind + (skills.length ? " · " + skills.length + " skills" : "");
      renderSkillOptions(skills);
    }
  } catch {
    setPill("pill-gateway", "gateway: offline", "err");
    setPill("pill-worker", "worker: unknown", "err");
  }
}

function renderSkillOptions(skills) {
  if (!skills || !skills.length) return;
  const signature = skills.join(",");
  if (signature === state.skillSignature) return;
  state.skillSignature = signature;
  const select = $("skill");
  const current = select.value;
  select.replaceChildren(
    ...skills.map((name) => el("option", { value: name, text: name })),
  );
  if (skills.includes(current)) select.value = current;
  else if (skills.length) select.value = skills[0];
}

/* ------------------------------------------------------------ simulate -- */

function specFor(profileId, seed) {
  const profile = PROFILES[profileId] || PROFILES.profile_so101_physx;
  return { ...profile, profile_id: profileId, seed };
}

function updateSpecPreview() {
  const profileId = $("profile").value;
  const seed = Math.max(0, Number($("seed").value || 1));
  $("spec-preview").textContent = JSON.stringify(specFor(profileId, seed), null, 2);
}

async function refreshSims() {
  try {
    const data = await api("/api/v1/simulations?limit=20");
    state.sims = data.simulations || [];
    renderSims();
    if (!state.simId) {
      const saved = localStorage.getItem("dsh.robotics.sim");
      const preferred =
        state.sims.find((sim) => sim.simulation_id === saved) ||
        state.sims.find((sim) => !sim.closed) ||
        state.sims[0];
      if (preferred) selectSim(preferred.simulation_id);
    }
  } catch {
    /* gateway offline: the status pill shows it */
  }
}

function renderSims() {
  const host = $("sim-list");
  host.replaceChildren();
  if (!state.sims.length) {
    host.append(el("div", { class: "empty small", text: "尚无仿真" }));
    return;
  }
  for (const sim of state.sims) {
    host.append(
      el(
        "button",
        {
          class: "sim-row" + (sim.simulation_id === state.simId ? " selected" : ""),
          title: sim.simulation_id,
          onclick: () => selectSim(sim.simulation_id),
        },
        [
          el("span", { class: "mono small", text: short(sim.simulation_id, 16) }),
          el("span", { class: "muted small", text: sim.closed ? "closed" : "live" }),
          el("span", { class: "muted small", text: timeOf(sim.created_at) }),
        ],
      ),
    );
  }
}

function selectSim(id) {
  if (!id) return;
  state.simId = id;
  localStorage.setItem("dsh.robotics.sim", id);
  state.selectedFrame = null;
  if (!DEPTH_DEFAULT) setDepthMode(false);
  $("viewer-empty").hidden = false;
  $("rgb").removeAttribute("src");
  $("frame-list").replaceChildren();
  $("frame-meta").hidden = true;
  $("frame-meta-line").textContent = "no frame";
  renderSims();
  updateControls();
  refreshState();
  refreshFrames();
}

async function startSimulation() {
  if (state.busy.has("start")) return;
  const profileId = $("profile").value;
  const seed = Math.max(0, Number($("seed").value || 1));
  state.busy.add("start");
  updateControls();
  try {
    const job = await api("/api/v1/simulations", { method: "POST", body: { spec: specFor(profileId, seed) } });
    toast("start job 已提交: " + short(job.job_id, 16), "info");
    await watchStartJob(job.job_id);
  } catch (error) {
    toast(error.message);
  } finally {
    state.busy.delete("start");
    updateControls();
    refreshJobs();
  }
}

async function watchStartJob(jobId) {
  const deadline = Date.now() + 15 * 60 * 1000;
  while (Date.now() < deadline) {
    await sleep(1500);
    let job;
    try {
      job = await api("/api/v1/jobs/" + encodeURIComponent(jobId));
    } catch {
      continue;
    }
    if (job.status === "succeeded" && job.simulation_id) {
      await refreshSims();
      selectSim(job.simulation_id);
      toast("仿真已启动: " + short(job.simulation_id), "info");
      return;
    }
    if (job.status === "failed" || job.status === "canceled") {
      const info = job.error || {};
      toast(`启动 ${job.status}: ${info.code || ""} ${info.message || ""}`);
      return;
    }
  }
  toast("启动超时：worker 可能仍在初始化，见 Jobs 面板");
}

async function resetSimulation() {
  if (!state.simId) return;
  try {
    await api(`/api/v1/simulations/${encodeURIComponent(state.simId)}/reset`, { method: "POST" });
    toast("已重置 episode", "info");
    refreshState();
    refreshSims();
  } catch (error) {
    toast(error.message);
  }
}

async function closeSimulation() {
  if (!state.simId) return;
  if (!window.confirm("关闭会释放 worker；若需再次启动而失败，需要重启 Gateway。继续？")) return;
  try {
    await api(`/api/v1/simulations/${encodeURIComponent(state.simId)}`, { method: "DELETE" });
    toast("已关闭仿真", "info");
    state.world = null;
    renderState();
    refreshSims();
    refreshJobs();
  } catch (error) {
    toast(error.message);
  }
}

/* --------------------------------------------------------------- world -- */

async function refreshState() {
  if (!state.simId) return;
  try {
    state.world = await api(`/api/v1/simulations/${encodeURIComponent(state.simId)}/state`);
    renderState();
  } catch (error) {
    if (error.code === "UNKNOWN_SIMULATION") {
      state.world = null;
      renderState();
    }
  }
}

function renderState() {
  const host = $("scene-meta");
  host.replaceChildren();
  const robots = $("robots");
  const objects = $("objects");
  const skills = $("skill-status");
  robots.replaceChildren();
  objects.replaceChildren();
  skills.replaceChildren();

  if (!state.world) {
    $("state-version").textContent = "—";
    host.append(el("div", { class: "empty small", text: "无活动仿真" }));
    return;
  }

  const world = state.world.state;
  $("state-version").textContent = "state_version " + world.state_version;

  const rows = [
    ["episode", world.episode_id],
    ["sim_step", world.sim_step],
    ["sim_time_s", fmt(world.sim_time_s, 2)],
    ["source", world.source],
    ["frame", world.frame],
  ];
  for (const [key, value] of rows) {
    host.append(el("span", { class: "k", text: key }), el("span", { class: "v", text: String(value) }));
  }

  for (const robot of world.robots || []) {
    const block = el("div", { class: "robot" }, [
      el("div", { class: "robot-head" }, [
        el("span", { class: "robot-name", text: robot.robot_id }),
        el("span", { class: "muted small", text: robot.dof + " dof" }),
      ]),
      el("div", { class: "kv" }, [
        el("span", { class: "k", text: "ee (m)" }),
        el("span", { class: "v", text: robot.ee_pose.position.map((v) => fmt(v, 3)).join(", ") }),
        el("span", { class: "k", text: "ee quat" }),
        el("span", { class: "v", text: robot.ee_pose.orientation_xyzw.map((v) => fmt(v, 3)).join(", ") }),
        el("span", { class: "k", text: "gripper" }),
        el("span", {
          class: "v",
          text: `${fmt(robot.gripper.width_m, 3)} m · ${robot.gripper.closed ? "closed" : "open"}`,
        }),
      ]),
      jointsTable(robot),
    ]);
    robots.append(block);
  }

  if ((world.objects || []).length) {
    objects.append(el("h3", { text: "对象" }));
    objects.append(
      el(
        "table",
        { class: "grid" },
        [
          el("thead", {}, [
            el("tr", {}, [
              el("th", { text: "id" }),
              el("th", { text: "position (m)" }),
              el("th", { text: "size (m)" }),
              el("th", { text: "state" }),
            ]),
          ]),
          el(
            "tbody",
            {},
            world.objects.map((object) =>
              el("tr", {}, [
                el("td", { text: object.object_id }),
                el("td", { text: object.pose.position.map((v) => fmt(v, 3)).join(", ") }),
                el("td", { text: object.size_m.map((v) => fmt(v, 3)).join(", ") }),
                el("td", {
                  text: (object.grasped_by ? "grasped by " + object.grasped_by : "free") + (object.visible ? "" : " · hidden"),
                }),
              ]),
            ),
          ),
        ],
      ),
    );
  }

  const skillStatuses = world.skills || [];
  if (skillStatuses.length) {
    skills.append(el("h3", { text: "技能状态" }));
    for (const status of skillStatuses) {
      skills.append(
        el("div", { class: "skill-row" }, [
          el("span", { class: "mono small", text: status.skill }),
          statusBadge(status.phase),
          progressBar(status.progress),
          status.message ? el("span", { class: "muted small", text: status.message }) : null,
        ]),
      );
    }
  }
}

function jointsTable(robot) {
  return el(
    "table",
    { class: "grid" },
    [
      el("thead", {}, [
        el("tr", {}, [el("th", { text: "#" }), el("th", { text: "q (rad)" }), el("th", { text: "qd (rad/s)" })]),
      ]),
      el(
        "tbody",
        {},
        robot.joint_positions.map((q, index) =>
          el("tr", {}, [
            el("td", { class: "muted", text: String(index) }),
            el("td", { text: fmt(q, 4) }),
            el("td", { text: fmt(robot.joint_velocities[index], 4) }),
          ]),
        ),
      ),
    ],
  );
}

/* --------------------------------------------------------------- jobs -- */

async function refreshJobs() {
  try {
    const data = await api("/api/v1/jobs?limit=40");
    state.jobs = data.jobs || [];
    renderJobs();
  } catch {
    /* quiet */
  }
}

function statusBadge(status) {
  return el("span", { class: "mono small status-" + status, text: status });
}

function progressBar(progress) {
  const value = Math.max(0, Math.min(1, Number(progress) || 0));
  return el("span", { class: "progress", title: Math.round(value * 100) + "%" }, [
    el("i", { style: "width:" + Math.round(value * 100) + "%" }),
  ]);
}

function renderJobs() {
  const body = $("jobs-body");
  body.replaceChildren();
  $("jobs-count").textContent = state.jobs.length ? state.jobs.length + " recent" : "";
  if (!state.jobs.length) {
    body.append(el("tr", {}, [el("td", { colspan: "7", class: "muted small", text: "暂无 job" })]));
    return;
  }
  for (const job of state.jobs) {
    const row = el(
      "tr",
      { class: "job-row" + (job.simulation_id === state.simId ? " current" : "") },
      [
        el("td", { class: "mono small", title: job.job_id, text: short(job.job_id, 13) }),
        el("td", { text: job.kind + (job.skill ? " · " + job.skill : "") }),
        el("td", { class: "mono small", title: job.simulation_id || "", text: job.simulation_id ? short(job.simulation_id, 10) : "—" }),
        el("td", {}, [statusBadge(job.status)]),
        el("td", {}, [progressBar(job.progress)]),
        el("td", { class: "muted small", text: timeOf(job.created_at) }),
        el("td", {}, [
          LIVE.has(job.status)
            ? el("button", {
                class: "small danger",
                text: "cancel",
                onclick: (event) => {
                  event.stopPropagation();
                  cancelJob(job.job_id);
                },
              })
            : null,
        ]),
      ],
    );
    row.addEventListener("click", () => showJob(job.job_id));
    body.append(row);
  }
}

async function showJob(jobId) {
  const box = $("job-detail");
  try {
    const job = await api("/api/v1/jobs/" + encodeURIComponent(jobId));
    box.textContent = JSON.stringify(job, null, 2);
    box.hidden = false;
  } catch (error) {
    toast(error.message);
  }
}

async function cancelJob(jobId) {
  try {
    await api("/api/v1/jobs/" + encodeURIComponent(jobId) + "/cancel", { method: "POST" });
    toast("已请求取消（等待 worker 确认）", "info");
    refreshJobs();
  } catch (error) {
    toast(error.message);
  }
}

/* --------------------------------------------------------------- frames -- */

async function refreshFrames() {
  if (!state.simId) return;
  try {
    const data = await api(`/api/v1/simulations/${encodeURIComponent(state.simId)}/frames?limit=12`);
    state.frames = data.frames || [];
  } catch {
    state.frames = [];
  }
  if (!state.selectedFrame && state.frames.length) showFrame(state.frames[0]);
  renderFrameList();
}

function renderFrameList() {
  const host = $("frame-list");
  host.replaceChildren();
  if (!state.frames.length) {
    host.append(el("span", { class: "muted small", text: "暂无帧" }));
    return;
  }
  for (const frame of state.frames) {
    host.append(
      el(
        "button",
        {
          class: "frame-chip" + (state.selectedFrame && frame.frame_id === state.selectedFrame.frame_id ? " selected" : ""),
          title: frame.frame_id,
          onclick: () => showFrame(frame),
        },
        [
          el("span", { class: "mono small", text: short(frame.frame_id, 10) }),
          el("span", { class: "muted small", text: "v" + frame.state_version + " · step " + frame.sim_step }),
        ],
      ),
    );
  }
}

async function captureFrame() {
  if (!state.simId) return toast("先选择仿真");
  if (state.busy.has("capture")) return;
  state.busy.add("capture");
  updateControls();
  try {
    const frame = await api(`/api/v1/simulations/${encodeURIComponent(state.simId)}/frames`, { method: "POST" });
    showFrame(frame);
    await refreshFrames();
    toast("已抓帧 " + short(frame.frame_id, 16), "info");
  } catch (error) {
    toast(error.message);
  } finally {
    state.busy.delete("capture");
    updateControls();
  }
}

function showFrame(frame) {
  state.selectedFrame = frame;
  $("viewer-empty").hidden = true;
  $("rgb").src = frame.rgb_url + "?t=" + Date.now();
  $("rgb").hidden = state.depthMode;
  $("depth").hidden = !state.depthMode;
  if (state.depthMode) loadDepth(frame);
  renderFrameMeta(frame);
  renderFrameList();
}

function renderFrameMeta(frame) {
  state.frameLine =
    `${short(frame.frame_id, 18)} · ${frame.width}×${frame.height} · state_version ${frame.state_version} · step ${frame.sim_step}`;
  $("frame-meta-line").textContent = state.frameLine;
  const meta = {
    frame_id: frame.frame_id,
    camera: frame.camera,
    width: frame.width,
    height: frame.height,
    episode_id: frame.episode_id,
    state_version: frame.state_version,
    sim_step: frame.sim_step,
    sim_time_s: frame.sim_time_s,
    fx: frame.fx,
    fy: frame.fy,
    cx: frame.cx,
    cy: frame.cy,
    capture_ms: frame.capture_ms,
    rgb_url: frame.rgb_url,
    depth_url: frame.depth_url,
  };
  $("frame-meta").textContent = JSON.stringify(meta, null, 2);
  $("frame-meta").hidden = false;
}

function setDepthMode(enabled) {
  state.depthMode = Boolean(enabled);
  $("tab-rgb").classList.toggle("active", !state.depthMode);
  $("tab-depth").classList.toggle("active", state.depthMode);
  if (state.selectedFrame) {
    $("rgb").hidden = state.depthMode;
    $("depth").hidden = !state.depthMode;
    if (state.depthMode) loadDepth(state.selectedFrame);
    else if (state.frameLine) $("frame-meta-line").textContent = state.frameLine;
  }
}

async function loadDepth(frame) {
  try {
    let parsed = state.depthCache.get(frame.frame_id);
    if (!parsed) {
      const response = await fetch(frame.depth_url);
      if (!response.ok) throw new Error("HTTP " + response.status);
      parsed = parseNpy(await response.arrayBuffer());
      state.depthCache.set(frame.frame_id, parsed);
    }
    drawDepth(parsed);
  } catch (error) {
    toast("depth: " + error.message);
    $("depth").hidden = true;
  }
}

function parseNpy(buffer) {
  const view = new DataView(buffer);
  const magic = String.fromCharCode(...new Uint8Array(buffer, 0, 6));
  if (magic !== "\x93NUMPY") throw new Error("not a .npy file");
  const major = view.getUint8(6);
  const headerLength = major === 1 ? view.getUint16(8, true) : view.getUint32(8, true);
  const headerOffset = major === 1 ? 10 : 12;
  const header = new TextDecoder("latin1").decode(new Uint8Array(buffer, headerOffset, headerLength));
  const descr = /'descr':\s*'([^']+)'/.exec(header);
  const shapeText = /'shape':\s*\(([^)]*)\)/.exec(header);
  if (!descr || descr[1] !== "<f4") throw new Error("unsupported dtype " + (descr ? descr[1] : "?"));
  if (!shapeText) throw new Error("missing shape");
  const shape = shapeText[1].split(",").map((part) => part.trim()).filter(Boolean).map(Number);
  if (!(shape.length === 2 || (shape.length === 3 && shape[2] === 1))) {
    throw new Error("depth must be 2-D (or H×W×1)");
  }
  const [height, width] = shape;
  const dataOffset = headerOffset + headerLength;
  if (dataOffset % 4 !== 0) throw new Error("misaligned float32 depth");
  const count = height * width;
  const data = new Float32Array(buffer, dataOffset, count);

  const valid = [];
  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < data.length; i += 1) {
    const value = data[i];
    if (Number.isFinite(value) && value > 0) {
      valid.push(value);
      if (value < min) min = value;
      if (value > max) max = value;
    }
  }
  valid.sort((a, b) => a - b);
  const lo = valid.length ? valid[Math.floor(valid.length * 0.02)] : 0;
  const hi = valid.length ? valid[Math.floor(valid.length * 0.98)] : 1;
  return {
    data,
    width,
    height,
    lo,
    hi,
    min: valid.length ? min : 0,
    max: valid.length ? max : 0,
    validRatio: data.length ? valid.length / data.length : 0,
  };
}

const TURBO = [
  [48, 18, 59],
  [70, 107, 227],
  [40, 187, 235],
  [121, 221, 112],
  [252, 224, 76],
  [231, 88, 45],
  [122, 4, 3],
];

function turbo(t) {
  const x = Math.max(0, Math.min(1, t)) * (TURBO.length - 1);
  const index = Math.min(TURBO.length - 2, Math.floor(x));
  const rest = x - index;
  const a = TURBO[index];
  const b = TURBO[index + 1];
  return [a[0] + (b[0] - a[0]) * rest, a[1] + (b[1] - a[1]) * rest, a[2] + (b[2] - a[2]) * rest];
}

function drawDepth(depth) {
  const canvas = $("depth");
  canvas.width = depth.width;
  canvas.height = depth.height;
  const context = canvas.getContext("2d");
  const image = context.createImageData(depth.width, depth.height);
  const range = depth.hi - depth.lo || 1;
  for (let i = 0; i < depth.data.length; i += 1) {
    const value = depth.data[i];
    const offset = i * 4;
    if (!Number.isFinite(value) || value <= 0) {
      image.data[offset] = 4;
      image.data[offset + 1] = 6;
      image.data[offset + 2] = 10;
      image.data[offset + 3] = 255;
      continue;
    }
    const color = turbo((value - depth.lo) / range);
    image.data[offset] = color[0];
    image.data[offset + 1] = color[1];
    image.data[offset + 2] = color[2];
    image.data[offset + 3] = 255;
  }
  context.putImageData(image, 0, 0);
  $("frame-meta-line").textContent =
    (state.frameLine || "") +
    ` · depth ${fmt(depth.min, 2)}–${fmt(depth.max, 2)} m · valid ${Math.round(depth.validRatio * 100)}%`;
}

/* --------------------------------------------------------------- skills -- */

function applySkillTemplate() {
  const template = SKILL_TEMPLATES[$("skill").value];
  if (!template) return;
  $("skill-params").value = JSON.stringify(template.parameters);
  $("timeout").value = template.timeout_s;
  $("expected").value = template.expected_duration_s;
}

async function submitSkill() {
  if (!state.simId) return toast("先选择仿真");
  const skill = $("skill").value;
  let parameters = {};
  const text = $("skill-params").value.trim();
  if (text) {
    try {
      parameters = JSON.parse(text);
    } catch {
      return toast("parameters 不是合法 JSON");
    }
  }
  const timeout_s = Number($("timeout").value || 30);
  const expected_duration_s = Number($("expected").value || 10);
  if (!(timeout_s > 0) || !(expected_duration_s > 0) || expected_duration_s > timeout_s) {
    return toast("要求 timeout_s ≥ expected_s > 0");
  }
  try {
    const job = await api(`/api/v1/simulations/${encodeURIComponent(state.simId)}/skills`, {
      method: "POST",
      body: { invocation: { skill, parameters, timeout_s, expected_duration_s, frame: "world" } },
    });
    toast("skill job 已提交: " + short(job.job_id, 16), "info");
    refreshJobs();
  } catch (error) {
    toast(error.message);
  }
}

/* ---------------------------------------------------------------- events -- */

function connectEvents() {
  if (state.ws) {
    try {
      state.ws.close();
    } catch {
      /* ignore */
    }
  }
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${protocol}://${location.host}/api/v1/events?after_seq=${state.lastSeq}`);
  state.ws = ws;
  ws.addEventListener("open", () => {
    $("ws-state").textContent = "live";
    $("ws-state").className = "muted small status-succeeded";
  });
  ws.addEventListener("message", (message) => {
    try {
      const record = JSON.parse(message.data);
      state.lastSeq = record.seq;
      appendEvent(record);
    } catch {
      /* ignore malformed frame */
    }
  });
  ws.addEventListener("close", () => {
    $("ws-state").textContent = "reconnecting…";
    $("ws-state").className = "muted small status-pending";
    setTimeout(connectEvents, 1500);
  });
  ws.addEventListener("error", () => {
    try {
      ws.close();
    } catch {
      /* ignore */
    }
  });
}

function appendEvent(record) {
  const log = $("events");
  const event = record.event || {};
  const stick = log.scrollTop + log.clientHeight >= log.scrollHeight - 12;
  log.append(
    el("div", { class: "event" + (record.simulation_id === state.simId ? " current" : "") }, [
      el("span", { text: String(record.seq).padStart(4, "0") }),
      el("span", { text: short(record.job_id, 11) }),
      el("span", { class: "event-type", text: event.type || "event" }),
      el("span", { text: event.status || "" }),
      el("span", { text: timeOf(record.created_at) }),
    ]),
  );
  while (log.childElementCount > 300) log.firstElementChild.remove();
  if (stick) log.scrollTop = log.scrollHeight;
}

/* --------------------------------------------------------------- layout -- */

function updateControls() {
  const hasSim = Boolean(state.simId);
  $("btn-reset").disabled = !hasSim;
  $("btn-close").disabled = !hasSim;
  $("btn-capture").disabled = !hasSim || state.busy.has("capture");
  $("btn-skill").disabled = !hasSim;
  $("btn-start").disabled = state.busy.has("start");
  $("btn-start").textContent = state.busy.has("start") ? "启动中…" : "启动仿真";
  $("btn-capture").textContent = state.busy.has("capture") ? "抓帧中…" : "抓帧";
}

function bind() {
  $("profile").addEventListener("change", updateSpecPreview);
  $("seed").addEventListener("input", updateSpecPreview);
  $("btn-start").addEventListener("click", startSimulation);
  $("btn-reset").addEventListener("click", resetSimulation);
  $("btn-capture").addEventListener("click", captureFrame);
  $("btn-close").addEventListener("click", closeSimulation);
  $("btn-skill").addEventListener("click", submitSkill);
  $("skill").addEventListener("change", applySkillTemplate);
  $("tab-rgb").addEventListener("click", () => setDepthMode(false));
  $("tab-depth").addEventListener("click", () => setDepthMode(true));
}

async function init() {
  bind();
  if (DEPTH_DEFAULT) setDepthMode(true);
  updateSpecPreview();
  applySkillTemplate();
  updateControls();
  try {
    await refreshStatus();
  } catch {
    /* the pills already show offline */
  }
  await refreshSims();
  await refreshJobs();
  connectEvents();
  setInterval(refreshStatus, 3000);
  setInterval(refreshSims, 3000);
  setInterval(refreshJobs, 2000);
  setInterval(() => {
    if (!document.hidden && state.simId) refreshState();
  }, 1200);
}

init();
