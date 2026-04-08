import * as ROSLIB from "roslib";
import { Terminal } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import "@xterm/xterm/css/xterm.css";
import { initRobotRenderUI } from "./robot-render.js";

// Basic config for ROS topics and legs/joints
const CONFIG = {
  ros: {
    bridgeUrl: "ws://10.12.64.222:9090",
    commandTopic: "/sam/command",
    testingTopic: "/sam/testing_state",
    webCommandTopic: "/web/command",
    globalPoseSequenceTopic: "/planner/global_pose_sequence",
    legCommandTopic: "/planner/leg_command",
    commandMessageType: "std_msgs/msg/String",
    testingMessageType: "std_msgs/msg/String",
    legCommandMessageType: "sam_interfaces/msg/LegCommand",
    imuMessageType: "sensor_msgs/Imu",
    imuRollPitchMessageType: "std_msgs/String",
    legEnabledStateMessageType: "std_msgs/msg/Bool",
    jetsonCpuTempTopic: "/jetson/cpu_temp",
    jetsonCpuLoadTopic: "/jetson/cpu_load_percent",
    float32MessageType: "std_msgs/msg/Float32",
    plannerStatusTopic: "/planner/status",
  },
  terminal: {
    gatewayUrl: "ws://localhost:8787",
  },
  legs: ["l0", "l1", "l2", "l3"],
  joints: ["inner_stepper", "outer_stepper", "servo"],
};
const DEFAULT_TERMINAL_PASSWORD = import.meta.env.VITE_TERMINAL_PASSWORD || "";
const ROS_BRIDGE_URL_STORAGE_KEY = "sam-ui-ros-bridge-url-v1";

/** Per-leg IMU topic paths (filtered IMU, String summary, debug raw). */
const legImuTopics = {
  l0: { imu: "/l0/imu/data", rollPitch: "/l0/imu/roll_pitch_deg", raw: "/l0/imu/data_raw" },
  l1: { imu: "/l1/imu/data", rollPitch: "/l1/imu/roll_pitch_deg", raw: "/l1/imu/data_raw" },
  l2: { imu: "/l2/imu/data", rollPitch: "/l2/imu/roll_pitch_deg", raw: "/l2/imu/data_raw" },
  l3: { imu: "/l3/imu/data", rollPitch: "/l3/imu/roll_pitch_deg", raw: "/l3/imu/data_raw" },
};

/**
 * Per-leg UI + ROS state (no shared global IMU store).
 * latestRollPitch parsed numbers feed the stance prism view.
 */
const legImuRegistry = Object.fromEntries(
  CONFIG.legs.map((id) => [
    id,
    {
      latestImDisplay: null,
      latestRollPitchReadout: null,
      latestRawDisplay: null,
      /** Latest values from `/lN/imu/roll_pitch_deg` before zero offset. */
      rollDegSensor: 0,
      pitchDegSensor: 0,
      /** Subtracted from sensor values so display / Render show 0° at the homed pose. */
      rollPitchZeroRoll: 0,
      rollPitchZeroPitch: 0,
      /** Displayed roll/pitch (sensor minus zero). */
      rollDeg: 0,
      pitchDeg: 0,
    },
  ])
);

let legImuRosTopics = [];
let legImuUiRaf = null;

let ros = null;
let commandTopic = null;
let testingTopic = null;
let webCommandTopic = null;
let poseSequenceTopic = null;
let legCommandTopic = null;
let trajectoryLegCommandRaf = null;
const trajectoryLegPending = new Map();
/** Latest enable flag per leg from `/{lN}/enabled_state` only (not from clicks). */
const legEnableStates = {};
const legEnableRosTopics = [];
let jetsonCpuTempTopic = null;
let jetsonCpuLoadTopic = null;
let plannerStatusRosTopic = null;
let isConnected = false;
let rosReconnectTimer = null;
let serviceSocket = null;
let serviceSocketUrl = "";
let terminalTabs = [];
let activeTabIndex = 0;
let terminalPanelMoved = false;
let deviceRefreshTimer = null;
let deviceScanInFlight = false;
let lastDeviceJson = "";

function $(id) {
  return document.getElementById(id);
}

function nowTime() {
  const d = new Date();
  return d.toLocaleTimeString(undefined, { hour12: false });
}

let activeLogFilter = "all";

function logLine(tag, message, kind = "info") {
  const log = $("log-output");
  if (!log) return;

  // Auto-assign ROS tag messages to "ros" kind unless they're errors
  if (tag === "ROS" && kind !== "error") {
    kind = "ros";
  }

  const line = document.createElement("div");
  line.className = "log-line";
  line.dataset.logKind = kind;

  if (activeLogFilter !== "all" && activeLogFilter !== kind) {
    line.style.display = "none";
  }

  const timeSpan = document.createElement("span");
  timeSpan.className = "log-time";
  timeSpan.textContent = nowTime();

  const tagSpan = document.createElement("span");
  const tagClass =
    kind === "error"
      ? "log-tag-error"
      : kind === "send"
      ? "log-tag-send"
      : kind === "ros"
      ? "log-tag-ros"
      : "log-tag-info";
  tagSpan.className = `log-tag ${tagClass}`;
  tagSpan.textContent = tag;

  const msgSpan = document.createElement("span");
  msgSpan.className = "log-message";
  msgSpan.textContent = message;

  line.appendChild(timeSpan);
  line.appendChild(tagSpan);
  line.appendChild(msgSpan);

  log.appendChild(line);
  log.scrollTop = log.scrollHeight;
}

function appendToTab(tabIndex, message) {
  const tab = terminalTabs[tabIndex];
  if (tab?.term) {
    tab.term.write(message);
  }
}

function getActiveTab() {
  return terminalTabs[activeTabIndex] ?? null;
}

function getFirstConnectedSocket() {
  for (const tab of terminalTabs) {
    if (tab.ws?.readyState === WebSocket.OPEN) return tab.ws;
  }
  return null;
}

function setStatus(status, kind = "disconnected") {
  const el = $("connection-status");
  if (!el) return;

  el.textContent = status;
  el.classList.remove(
    "status-connected",
    "status-error",
    "status-disconnected"
  );

  if (kind === "connected") {
    el.classList.add("status-connected");
  } else if (kind === "error") {
    el.classList.add("status-error");
  } else {
    el.classList.add("status-disconnected");
  }
}

function setTerminalStatus(status, kind = "disconnected") {
  const el = $("terminal-connection-status");
  if (!el) return;

  el.textContent = status;
  el.classList.remove("status-connected", "status-error", "status-disconnected");

  if (kind === "connected") {
    el.classList.add("status-connected");
  } else if (kind === "error") {
    el.classList.add("status-error");
  } else {
    el.classList.add("status-disconnected");
  }
}

function teardownTrajectoryTopics() {
  if (poseSequenceTopic) {
    try {
      poseSequenceTopic.unsubscribe();
    } catch (_e) {
      // ignore
    }
    poseSequenceTopic = null;
  }
  if (legCommandTopic) {
    try {
      legCommandTopic.unsubscribe();
    } catch (_e) {
      // ignore
    }
    legCommandTopic = null;
  }
  if (trajectoryLegCommandRaf != null) {
    cancelAnimationFrame(trajectoryLegCommandRaf);
    trajectoryLegCommandRaf = null;
  }
  trajectoryLegPending.clear();
}

function teardownLegEnableTopics() {
  for (const topic of legEnableRosTopics) {
    try {
      topic.unsubscribe();
    } catch (_e) {
      // ignore
    }
  }
  legEnableRosTopics.length = 0;
  for (const id of Object.keys(legEnableStates)) {
    delete legEnableStates[id];
  }
  for (const { id } of LEGS) {
    const el = document.querySelector(`[data-leg-enable-display="${id}"]`);
    if (el) el.textContent = "\u2014";
  }
}

function teardownImuTopics() {
  for (const t of legImuRosTopics) {
    try {
      t.unsubscribe();
    } catch (_e) {
      // ignore
    }
  }
  legImuRosTopics = [];
  if (legImuUiRaf != null) {
    cancelAnimationFrame(legImuUiRaf);
    legImuUiRaf = null;
  }
  for (const id of CONFIG.legs) {
    const r = legImuRegistry[id];
    r.latestImDisplay = null;
    r.latestRollPitchReadout = null;
    r.latestRawDisplay = null;
    r.rollDegSensor = 0;
    r.pitchDegSensor = 0;
    r.rollPitchZeroRoll = 0;
    r.rollPitchZeroPitch = 0;
    r.rollDeg = 0;
    r.pitchDeg = 0;
  }
}

function teardownJetsonLoadTopics() {
  if (jetsonCpuTempTopic) {
    try {
      jetsonCpuTempTopic.unsubscribe();
    } catch (_e) {
      // ignore
    }
    jetsonCpuTempTopic = null;
  }
  if (jetsonCpuLoadTopic) {
    try {
      jetsonCpuLoadTopic.unsubscribe();
    } catch (_e) {
      // ignore
    }
    jetsonCpuLoadTopic = null;
  }
  resetJetsonLoadDisplays();
}

function teardownPlannerStatusTopics() {
  if (plannerStatusRosTopic) {
    try {
      plannerStatusRosTopic.unsubscribe();
    } catch (_e) {
      // ignore
    }
    plannerStatusRosTopic = null;
  }
  resetPlannerStatusDisplay();
}

function cleanupRosState() {
  teardownTrajectoryTopics();
  teardownImuTopics();
  teardownLegEnableTopics();
  teardownJetsonLoadTopics();
  teardownPlannerStatusTopics();
  isConnected = false;
  commandTopic = null;
  testingTopic = null;
  webCommandTopic = null;
  ros = null;
}

function scheduleRosReconnect() {
  if (rosReconnectTimer || isConnected) return;
  setStatus("Reconnecting...", "disconnected");
  rosReconnectTimer = setTimeout(() => {
    rosReconnectTimer = null;
    connectRos();
  }, 2000);
}

function connectRos() {
  const url = CONFIG.ros.bridgeUrl;

  if (isConnected) return;

  if (ros) {
    try {
      ros.close();
    } catch (_err) {
      // ignore close errors
    }
    cleanupRosState();
  }

  try {
    ros = new ROSLIB.Ros({ url });
  } catch (e) {
    logLine("ROS", "Failed to create ROS connection object: " + e.message, "error");
    setStatus("Connection error", "error");
    scheduleRosReconnect();
    return;
  }

  setStatus("Connecting...", "disconnected");

  ros.on("connection", () => {
    isConnected = true;
    setStatus("Connected", "connected");
    console.log("[ros] WebSocket connected to", url);
    logLine("ROS", "Connected to " + url);
    const lb = $("launch-ros-bridge-btn");
    if (lb) lb.hidden = true;
    initTopics();
  });

  ros.on("error", (err) => {
    setStatus("Connection error", "error");
    logLine("ROS", "Error: " + (err?.message || String(err)), "error");
  });

  ros.on("close", () => {
    const wasConnected = isConnected;
    cleanupRosState();
    setStatus("Disconnected", "disconnected");
    const lb = $("launch-ros-bridge-btn");
    if (lb) lb.hidden = false;
    logLine("ROS", wasConnected ? "Connection closed" : "Disconnected");
    scheduleRosReconnect();
  });
}

function setupRos() {
  const endpoint = $("ros-endpoint-value");
  const bridgeSelect = $("ros-bridge-select");
  const rosMenuBtn = $("ros-menu-btn");
  const rosMetaPanel = $("ros-meta-panel");

  // Restore last-used rosbridge URL (if present).
  try {
    const saved = localStorage.getItem(ROS_BRIDGE_URL_STORAGE_KEY);
    if (saved && typeof saved === "string") {
      CONFIG.ros.bridgeUrl = saved;
    }
  } catch (_err) {
    // ignore
  }

  if (endpoint) {
    endpoint.textContent = CONFIG.ros.bridgeUrl;
  }

  if (bridgeSelect) {
    bridgeSelect.value = CONFIG.ros.bridgeUrl;
    bridgeSelect.addEventListener("change", () => {
      const next = String(bridgeSelect.value || "").trim();
      if (!next || next === CONFIG.ros.bridgeUrl) return;
      CONFIG.ros.bridgeUrl = next;
      if (endpoint) endpoint.textContent = next;
      try {
        localStorage.setItem(ROS_BRIDGE_URL_STORAGE_KEY, next);
      } catch (_err) {
        // ignore
      }
      // Force reconnect to the newly selected rosbridge.
      try {
        ros?.close();
      } catch (_err) {
        // ignore
      }
      cleanupRosState();
      connectRos();
    });
  }

  if (rosMenuBtn && rosMetaPanel) {
    rosMenuBtn.addEventListener("click", () => {
      const shouldShow = rosMetaPanel.hidden;
      rosMetaPanel.hidden = !shouldShow;
      rosMenuBtn.setAttribute("aria-expanded", String(shouldShow));
    });

    document.addEventListener("click", (event) => {
      if (rosMetaPanel.hidden) return;
      const target = event.target;
      if (!(target instanceof Node)) return;
      if (rosMetaPanel.contains(target) || rosMenuBtn.contains(target)) return;
      rosMetaPanel.hidden = true;
      rosMenuBtn.setAttribute("aria-expanded", "false");
    });
  }

  connectRos();
}

function initTopics() {
  if (!ros) return;

  commandTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.commandTopic,
    messageType: CONFIG.ros.commandMessageType,
  });

  testingTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.testingTopic,
    messageType: CONFIG.ros.testingMessageType,
  });

  webCommandTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.webCommandTopic,
    messageType: CONFIG.ros.commandMessageType,
  });

  console.log("[ros] Topics created:", {
    commandTopic: { name: commandTopic.name, messageType: commandTopic.messageType },
    testingTopic: { name: testingTopic.name, messageType: testingTopic.messageType },
    webCommandTopic: { name: webCommandTopic.name, messageType: webCommandTopic.messageType },
  });
  logLine("ROS", `Topics initialized — /web/command as ${CONFIG.ros.commandMessageType}`, "ros");
  initTrajectoryTopics();
  initGyroTopics();
  initLegEnableTopics();
  initJetsonLoadTopics();
  initPlannerStatusTopics();
}

function resetPlannerStatusDisplay() {
  const stateEl = $("planner-state");
  const goalEl = $("planner-goal");
  const progressEl = $("planner-progress");
  const progressBar = $("planner-progress-bar");
  if (stateEl) {
    stateEl.textContent = "\u2014";
    stateEl.className = "planner-state-idle";
  }
  if (goalEl) goalEl.textContent = "\u2014";
  if (progressEl) progressEl.textContent = "\u2014";
  if (progressBar) progressBar.style.width = "0%";
}

function updatePlannerStatusDisplay(status) {
  if (!status || typeof status !== "object") return;

  const stateRaw = status.state;
  const stateKey =
    typeof stateRaw === "string" && stateRaw.length > 0 ? stateRaw.toLowerCase() : "unknown";

  const stateEl = $("planner-state");
  if (stateEl) {
    stateEl.textContent = stateKey.toUpperCase();
    stateEl.className = "planner-state-" + stateKey;
  }

  const goalEl = $("planner-goal");
  if (goalEl) {
    const g = status.goal;
    if (g && g.type != null && g.count != null) {
      goalEl.textContent = `${g.type} x${g.count}`;
    } else {
      goalEl.textContent = "\u2014";
    }
  }

  const pctRaw = status.progress_percent;
  const nPoses = status.total_poses;
  const pctNum =
    typeof pctRaw === "number" && Number.isFinite(pctRaw)
      ? pctRaw
      : Number.parseFloat(pctRaw);
  const pct = Number.isFinite(pctNum) ? pctNum : 0;
  const posesNum =
    typeof nPoses === "number" && Number.isFinite(nPoses)
      ? nPoses
      : Number.parseInt(String(nPoses ?? ""), 10);
  const poses = Number.isFinite(posesNum) ? posesNum : 0;

  const progressEl = $("planner-progress");
  if (progressEl) {
    progressEl.textContent = `${pct}% (${poses} poses)`;
  }

  const progressBar = $("planner-progress-bar");
  if (progressBar) {
    const w = Math.min(100, Math.max(0, pct));
    progressBar.style.width = `${w}%`;
  }
}

function initPlannerStatusTopics() {
  if (!ros) return;
  teardownPlannerStatusTopics();

  plannerStatusRosTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.plannerStatusTopic,
    messageType: CONFIG.ros.commandMessageType,
  });

  plannerStatusRosTopic.subscribe((message) => {
    try {
      const raw = message?.data;
      if (raw == null || String(raw).trim() === "") return;
      const plannerStatus = JSON.parse(raw);
      updatePlannerStatusDisplay(plannerStatus);

      const prog = plannerStatus.progress_percent;
      console.log("Progress:", `${prog ?? 0}%`);
      console.log("Planner Status:", {
        state: plannerStatus.state,
        goal: plannerStatus.goal,
        total_poses: plannerStatus.total_poses,
        progress: `${prog ?? 0}%`,
      });
      if (plannerStatus.state === "idle") {
        console.log("Movement complete!");
      }
    } catch (e) {
      console.error("Failed to parse planner status:", e);
    }
  });

  console.log("[ros] Planner status:", CONFIG.ros.plannerStatusTopic);
}

/** Per-leg ROS topic: `/l0/enabled_state`, `/l1/enabled_state`, … from CONFIG.legs */
function legEnabledStateTopicPathForIndex(index) {
  const legKey = CONFIG.legs[index];
  return legKey ? `/${legKey}/enabled_state` : "";
}

function updateLegStatusDisplay(legId, value) {
  const el = document.querySelector(`[data-leg-enable-display="${legId}"]`);
  if (!el) return;
  el.textContent = String(value);
}

function initLegEnableTopics() {
  if (!ros) return;
  teardownLegEnableTopics();

  LEGS.forEach((legDef, index) => {
    const topicName = legEnabledStateTopicPathForIndex(index);
    if (!topicName) return;

    const displayId = legDef.id;
    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: CONFIG.ros.legEnabledStateMessageType,
    });
    legEnableRosTopics.push(topic);
    topic.subscribe((message) => {
      const value = message?.data;
      if (typeof value !== "boolean") return;
      legEnableStates[displayId] = value;
      updateLegStatusDisplay(displayId, value);
    });
  });

  console.log(
    "[ros] Leg enable state topics (per leg):",
    LEGS.map((_leg, i) => legEnabledStateTopicPathForIndex(i)).filter(Boolean)
  );
}

function sendLegEnableToggleCommand(legId) {
  const current = legEnableStates[legId];
  const next = current === true ? false : true;
  const payload = { leg_id: legId, type: "enable", value: next };
  publishPayload(JSON.stringify(payload));
}

function resetJetsonLoadDisplays() {
  const tempEl = $("cpu-temp-display");
  const loadEl = $("cpu-load-display");
  if (tempEl) {
    tempEl.textContent = "\u2014";
    tempEl.className = "jetson-metric-value temp-normal";
  }
  if (loadEl) {
    loadEl.textContent = "\u2014";
    loadEl.className = "jetson-metric-value load-idle";
  }
}

function updateCpuTempDisplay(tempStr, numericTemp) {
  const element = $("cpu-temp-display");
  if (!element) return;
  element.textContent = `${tempStr}\u00b0C`;
  element.className =
    numericTemp > 70 ? "jetson-metric-value temp-warning" : "jetson-metric-value temp-normal";
}

function updateCpuLoadDisplay(loadStr, numericLoad) {
  const element = $("cpu-load-display");
  if (!element) return;
  element.textContent = `${loadStr}%`;
  let band = "load-high";
  if (numericLoad < 50) band = "load-low";
  else if (numericLoad < 75) band = "load-medium";
  element.className = `jetson-metric-value ${band}`;
}

function initJetsonLoadTopics() {
  if (!ros) return;
  teardownJetsonLoadTopics();

  jetsonCpuTempTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.jetsonCpuTempTopic,
    messageType: CONFIG.ros.float32MessageType,
  });
  jetsonCpuTempTopic.subscribe((message) => {
    const raw = message?.data;
    const n = typeof raw === "number" ? raw : Number.parseFloat(raw);
    if (!Number.isFinite(n)) return;
    const valueStr = n.toFixed(1);
    console.log("CPU Temp:", `${valueStr}\u00b0C`);
    updateCpuTempDisplay(valueStr, n);
  });

  jetsonCpuLoadTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.jetsonCpuLoadTopic,
    messageType: CONFIG.ros.float32MessageType,
  });
  jetsonCpuLoadTopic.subscribe((message) => {
    const raw = message?.data;
    const n = typeof raw === "number" ? raw : Number.parseFloat(raw);
    if (!Number.isFinite(n)) return;
    const valueStr = n.toFixed(1);
    console.log("CPU Load:", `${valueStr}%`);
    updateCpuLoadDisplay(valueStr, n);
  });

  console.log("[ros] Jetson load topics:", {
    cpuTemp: CONFIG.ros.jetsonCpuTempTopic,
    cpuLoad: CONFIG.ros.jetsonCpuLoadTopic,
  });
}

function normalizePlannerLegIndex(message) {
  const raw = message?.leg_id ?? message?.legId;
  if (typeof raw === "number" && Number.isFinite(raw)) return raw;
  if (typeof raw === "string") {
    const m = raw.match(/(\d+)/);
    if (m) return Number.parseInt(m[1], 10);
  }
  return NaN;
}

function flushTrajectoryLegDisplays() {
  trajectoryLegCommandRaf = null;
  trajectoryLegPending.forEach((msg, legIndex) => {
    const legKey = CONFIG.legs[legIndex];
    if (!legKey) return;
    const pre = document.querySelector(`[data-trajectory-leg="${legKey}"]`);
    if (pre) {
      try {
        pre.textContent = JSON.stringify(msg, null, 2);
      } catch (_e) {
        pre.textContent = String(msg);
      }
    }
  });
  trajectoryLegPending.clear();
}

function initTrajectoryTopics() {
  if (!ros) return;
  teardownTrajectoryTopics();

  poseSequenceTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.globalPoseSequenceTopic,
    messageType: CONFIG.ros.commandMessageType,
  });

  poseSequenceTopic.subscribe((message) => {
    const el = $("trajectory-pose-sequence");
    if (!el) return;
    const raw = message?.data ?? "";
    try {
      const data = JSON.parse(raw);
      el.textContent = JSON.stringify(data, null, 2);
      console.log("Trajectory:", data);
    } catch (_e) {
      el.textContent = raw;
    }
  });

  legCommandTopic = new ROSLIB.Topic({
    ros,
    name: CONFIG.ros.legCommandTopic,
    messageType: CONFIG.ros.legCommandMessageType,
  });

  legCommandTopic.subscribe((message) => {
    const legIndex = normalizePlannerLegIndex(message);
    if (!Number.isFinite(legIndex) || legIndex < 0 || legIndex >= CONFIG.legs.length) {
      return;
    }
    trajectoryLegPending.set(legIndex, message);
    if (trajectoryLegCommandRaf != null) return;
    trajectoryLegCommandRaf = requestAnimationFrame(flushTrajectoryLegDisplays);
  });

  console.log("[ros] Trajectory topics:", {
    globalPoseSequence: CONFIG.ros.globalPoseSequenceTopic,
    legCommand: CONFIG.ros.legCommandTopic,
  });
}

/** Quaternion (geometry_msgs convention x,y,z,w) to roll, pitch, yaw in radians (ZYX / yaw-pitch-roll order). */
function quatToRollPitchYawRad(q) {
  const x = Number(q?.x);
  const y = Number(q?.y);
  const z = Number(q?.z);
  const w = Number(q?.w);
  if (![x, y, z, w].every((v) => Number.isFinite(v))) {
    return { roll: NaN, pitch: NaN, yaw: NaN };
  }
  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);
  const sinp = 2 * (w * y - z * x);
  let pitch;
  if (Math.abs(sinp) >= 1) {
    pitch = Math.PI / 2 * Math.sign(sinp);
  } else {
    pitch = Math.asin(sinp);
  }
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);
  return { roll, pitch, yaw };
}

function radToDeg(r) {
  if (!Number.isFinite(r)) return null;
  return (r * 180) / Math.PI;
}

function formatFilteredImuForDisplay(message, topicName) {
  const ori = message?.orientation ?? {};
  const la = message?.linear_acceleration ?? {};
  const av = message?.angular_velocity ?? {};
  const quat = { x: ori.x, y: ori.y, z: ori.z, w: ori.w };
  const rpy = quatToRollPitchYawRad(quat);
  const fmtDeg = (rad) => {
    const d = radToDeg(rad);
    return d == null ? null : Number(d.toFixed(4));
  };
  const payload = {
    topic: topicName,
    frame_id: message?.header?.frame_id ?? "",
    orientation_quaternion: { x: ori.x, y: ori.y, z: ori.z, w: ori.w },
    orientation_euler_deg: {
      roll_deg: fmtDeg(rpy.roll),
      pitch_deg: fmtDeg(rpy.pitch),
      yaw_deg: fmtDeg(rpy.yaw),
    },
    angular_velocity: { x: av.x, y: av.y, z: av.z },
    linear_acceleration: { x: la.x, y: la.y, z: la.z },
  };
  return JSON.stringify(payload, null, 2);
}

/** Parse roll_pitch_deg string: roll_deg=..,pitch_deg=.., JSON, or two numbers. */
function parseRollPitchDegString(raw) {
  const s = raw != null ? String(raw).trim() : "";
  if (!s) return { summary: "—", roll_deg: null, pitch_deg: null };
  const keyVal = s.match(
    /roll_deg\s*=\s*(-?\d+\.?\d*)\s*,\s*pitch_deg\s*=\s*(-?\d+\.?\d*)/i
  );
  if (keyVal) {
    const rd = Number(keyVal[1]);
    const pd = Number(keyVal[2]);
    if (Number.isFinite(rd) && Number.isFinite(pd)) {
      return {
        summary: `roll ${rd.toFixed(2)}° · pitch ${pd.toFixed(2)}°`,
        roll_deg: rd,
        pitch_deg: pd,
      };
    }
  }
  try {
    const j = JSON.parse(s);
    if (j && typeof j === "object") {
      const rd = j.roll_deg ?? j.roll ?? j.rollDeg;
      const pd = j.pitch_deg ?? j.pitch ?? j.pitchDeg;
      if (Number.isFinite(Number(rd)) && Number.isFinite(Number(pd))) {
        return {
          summary: `roll ${Number(rd).toFixed(2)}° · pitch ${Number(pd).toFixed(2)}°`,
          roll_deg: Number(rd),
          pitch_deg: Number(pd),
        };
      }
    }
  } catch (_e) {
    // not JSON
  }
  const numPair = s.match(/(-?\d+\.?\d*)[^\d.+-]+(-?\d+\.?\d*)/);
  if (numPair) {
    const rd = Number(numPair[1]);
    const pd = Number(numPair[2]);
    if (Number.isFinite(rd) && Number.isFinite(pd)) {
      return {
        summary: `roll ${rd.toFixed(2)}° · pitch ${pd.toFixed(2)}°`,
        roll_deg: rd,
        pitch_deg: pd,
      };
    }
  }
  return { summary: s, roll_deg: null, pitch_deg: null };
}

function applyRollPitchDisplayForLeg(legId) {
  const r = legImuRegistry[legId];
  if (!r) return;
  const sr = r.rollDegSensor;
  const sp = r.pitchDegSensor;
  if (!Number.isFinite(sr) || !Number.isFinite(sp)) return;
  r.rollDeg = sr - r.rollPitchZeroRoll;
  r.pitchDeg = sp - r.rollPitchZeroPitch;
  r.latestRollPitchReadout = `roll ${r.rollDeg.toFixed(2)}\u00b0 \u00b7 pitch ${r.pitchDeg.toFixed(2)}\u00b0`;
}

/** Set current sensor roll/pitch as the new 0° reference (display, stance viz, 3D Render). */
function zeroImuRollPitchDisplay() {
  for (const id of CONFIG.legs) {
    const r = legImuRegistry[id];
    r.rollPitchZeroRoll = Number.isFinite(r.rollDegSensor) ? r.rollDegSensor : 0;
    r.rollPitchZeroPitch = Number.isFinite(r.pitchDegSensor) ? r.pitchDegSensor : 0;
    applyRollPitchDisplayForLeg(id);
  }
  scheduleLegImuUiUpdate();
  logLine("GYRO", "Roll/pitch display zeroed (current attitude is now 0° reference per leg)", "info");
}

function scheduleLegImuUiUpdate() {
  if (legImuUiRaf != null) return;
  legImuUiRaf = requestAnimationFrame(flushLegImuUi);
}

function flushLegImuUi() {
  legImuUiRaf = null;
  for (const id of CONFIG.legs) {
    const r = legImuRegistry[id];
    const filteredEl = document.querySelector(`[data-leg-imu-filtered="${id}"]`);
    if (filteredEl) filteredEl.textContent = r.latestImDisplay ?? "\u2014";
    const rpEl = document.querySelector(`[data-leg-roll-pitch="${id}"]`);
    if (rpEl) rpEl.textContent = r.latestRollPitchReadout ?? "\u2014";
    const rawEl = document.querySelector(`[data-leg-imu-raw="${id}"]`);
    if (rawEl) rawEl.textContent = r.latestRawDisplay ?? "\u2014";

    const rot = document.querySelector(`[data-leg-stance-rot="${id}"]`);
    if (rot) {
      rot.style.setProperty("--leg-roll-num", String(Number.isFinite(r.rollDeg) ? r.rollDeg : 0));
      rot.style.setProperty("--leg-pitch-num", String(Number.isFinite(r.pitchDeg) ? r.pitchDeg : 0));
    }
    const label = document.querySelector(`[data-leg-stance-label="${id}"]`);
    if (label) {
      label.textContent = `${id}  r ${Number(r.rollDeg).toFixed(1)}\u00b0  p ${Number(r.pitchDeg).toFixed(1)}\u00b0`;
    }
  }
}

function initGyroTopics() {
  if (!ros) return;
  teardownImuTopics();

  const imuType = CONFIG.ros.imuMessageType;
  const rpType = CONFIG.ros.imuRollPitchMessageType;

  for (const legId of CONFIG.legs) {
    const paths = legImuTopics[legId];
    if (!paths) continue;

    const tImu = new ROSLIB.Topic({
      ros,
      name: paths.imu,
      messageType: imuType,
    });
    tImu.subscribe((message) => {
      legImuRegistry[legId].latestImDisplay = formatFilteredImuForDisplay(message, paths.imu);
      scheduleLegImuUiUpdate();
    });
    legImuRosTopics.push(tImu);

    const tRp = new ROSLIB.Topic({
      ros,
      name: paths.rollPitch,
      messageType: rpType,
    });
    tRp.subscribe((message) => {
      const str = message?.data != null ? String(message.data) : "";
      const parsed = parseRollPitchDegString(str);
      if (parsed.roll_deg != null && parsed.pitch_deg != null) {
        const r = legImuRegistry[legId];
        r.rollDegSensor = parsed.roll_deg;
        r.pitchDegSensor = parsed.pitch_deg;
        applyRollPitchDisplayForLeg(legId);
      } else {
        legImuRegistry[legId].latestRollPitchReadout = parsed.summary;
      }
      scheduleLegImuUiUpdate();
    });
    legImuRosTopics.push(tRp);

    const tRaw = new ROSLIB.Topic({
      ros,
      name: paths.raw,
      messageType: imuType,
    });
    tRaw.subscribe((message) => {
      legImuRegistry[legId].latestRawDisplay = formatFilteredImuForDisplay(message, paths.raw);
      scheduleLegImuUiUpdate();
    });
    legImuRosTopics.push(tRaw);
  }

  scheduleLegImuUiUpdate();
  console.log("[ros] Per-leg IMU topics", { imuType, rpType, legImuTopics });
}


const CMD_FIELDS = {
  move:   ["inner", "outer", "servo"],
  servo:  ["value"],
  enable: ["value"],
  query:  [],
  home:   [],
  rate:   ["value"],
  raw:    ["command"],
  walk:   ["distance"],
};

const CMD_DEFAULTS = {
  inner: 0, outer: 0, servo: 0, value: 0, command: "", distance: "5",
};

function buildCmdFields() {
  const type = $("cmd-type").value;
  const legGroup = $("cmd-leg-group");
  if (legGroup) legGroup.hidden = type === "move";
  const container = $("cmd-fields");
  container.innerHTML = "";

  (CMD_FIELDS[type] || []).forEach((field) => {
    const group = document.createElement("div");
    group.className = "field-group";

    const lbl = document.createElement("label");
    lbl.setAttribute("for", `cmd-f-${field}`);
    lbl.textContent = field;

    let input;
    if (type === "enable" && field === "value") {
      input = document.createElement("select");
      input.id = `cmd-f-${field}`;
      const optTrue = document.createElement("option");
      optTrue.value = "true";
      optTrue.textContent = "true";
      const optFalse = document.createElement("option");
      optFalse.value = "false";
      optFalse.textContent = "false";
      input.appendChild(optTrue);
      input.appendChild(optFalse);
    } else if (field === "command") {
      input = document.createElement("input");
      input.type = "text";
      input.id = `cmd-f-${field}`;
      input.placeholder = "e.g. A1E";
    } else {
      input = document.createElement("input");
      input.type = "number";
      input.step = "any";
      input.id = `cmd-f-${field}`;
      input.value = CMD_DEFAULTS[field] ?? 0;
      input.addEventListener("focus", () => input.select());
    }

    group.appendChild(lbl);
    group.appendChild(input);
    container.appendChild(group);
  });

  updateCmdPreview();
}

function buildCmdPayload() {
  const type = $("cmd-type").value;
  const leg = $("cmd-leg")?.value ?? "l0";

  if (type === "walk") {
    const el = document.getElementById("cmd-f-distance");
    const distance = el ? String(el.value || "").trim() || "5" : "5";
    return { type: "walk", distance };
  }

  if (type === "move") {
    const inner = parseFloat(document.getElementById("cmd-f-inner")?.value) || 0;
    const outer = parseFloat(document.getElementById("cmd-f-outer")?.value) || 0;
    const servo = parseFloat(document.getElementById("cmd-f-servo")?.value) || 0;
    return { type: "move", allLegs: true, inner, outer, servo };
  }

  const payload = { type, leg_id: leg };

  (CMD_FIELDS[type] || []).forEach((field) => {
    const el = document.getElementById(`cmd-f-${field}`);
    if (!el) return;
    const raw = el.value;

    if (type === "enable" && field === "value") {
      payload[field] = raw === "true";
    } else if (field === "command") {
      payload[field] = raw;
    } else {
      payload[field] = parseFloat(raw) || 0;
    }
  });

  return payload;
}

function updateCmdPreview() {}

/** Saved command chips (Commands panel); persisted in localStorage. */
const SAVED_CMDS_STORAGE_KEY = "sam-ui-saved-commands-v1";

function persistSavedCommands() {
  const container = $("cmd-saved");
  if (!container) return;
  const items = [];
  for (const wrapper of container.querySelectorAll(".cmd-sticky")) {
    const btn = wrapper.querySelector(".cmd-sticky-btn");
    if (!btn) continue;
    const name = (btn.textContent || "").trim();
    const cmd = btn.dataset.cmd;
    if (name && typeof cmd === "string" && cmd.length > 0) items.push({ name, cmd });
  }
  try {
    localStorage.setItem(SAVED_CMDS_STORAGE_KEY, JSON.stringify(items));
  } catch (err) {
    console.warn("[cmd] Could not persist saved commands", err);
    logLine("CMD", "Could not write saved commands to browser storage", "error");
  }
}

function restoreSavedCommands() {
  const container = $("cmd-saved");
  if (!container) return;
  let raw;
  try {
    raw = localStorage.getItem(SAVED_CMDS_STORAGE_KEY);
  } catch (err) {
    console.warn("[cmd] localStorage unavailable", err);
    return;
  }
  if (!raw) return;
  let items;
  try {
    items = JSON.parse(raw);
  } catch {
    return;
  }
  if (!Array.isArray(items)) return;
  for (const it of items) {
    if (!it || typeof it.name !== "string" || typeof it.cmd !== "string") continue;
    try {
      JSON.parse(it.cmd);
    } catch {
      continue;
    }
    installSavedCommandChip(it.name.trim(), it.cmd, false);
  }
}

/**
 * @param {string} name
 * @param {string} cmdJsonStr
 * @param {boolean} [doPersist=true]
 */
function installSavedCommandChip(name, cmdJsonStr, doPersist = true) {
  const container = $("cmd-saved");
  if (!container) return;

  const wrapper = document.createElement("div");
  wrapper.className = "cmd-sticky";

  const btn = document.createElement("button");
  btn.type = "button";
  btn.className = "cmd-sticky-btn";
  btn.textContent = name;
  btn.dataset.cmd = cmdJsonStr;

  btn.addEventListener("click", () => {
    let parsed;
    try {
      parsed = JSON.parse(btn.dataset.cmd);
    } catch {
      logLine("CMD", "Invalid JSON in saved command", "error");
      return;
    }
    if (parsed.allLegs && parsed.type === "move") {
      const { inner, outer, servo } = parsed;
      for (const leg of CONFIG.legs) {
        publishPayload(JSON.stringify({ type: "move", leg_id: leg, inner, outer, servo }));
      }
    } else {
      publishPayload(btn.dataset.cmd);
    }
  });

  btn.addEventListener("contextmenu", (e) => {
    e.preventDefault();
    const newJson = prompt("Edit command JSON:", btn.dataset.cmd);
    if (newJson !== null) {
      try {
        JSON.parse(newJson);
        btn.dataset.cmd = newJson;
        logLine("CMD", `Updated "${name}": ${newJson}`);
        persistSavedCommands();
      } catch {
        logLine("CMD", "Invalid JSON, not saved", "error");
      }
    }
  });

  const removeBtn = document.createElement("button");
  removeBtn.type = "button";
  removeBtn.className = "cmd-sticky-remove";
  removeBtn.textContent = "\u00d7";
  removeBtn.title = "Remove";
  removeBtn.addEventListener("click", () => {
    wrapper.remove();
    persistSavedCommands();
  });

  wrapper.appendChild(btn);
  wrapper.appendChild(removeBtn);
  container.appendChild(wrapper);
  if (doPersist) persistSavedCommands();
}

const cmdHistory = [];
let cmdHistoryIdx = -1;
let cmdHistoryDraft = "";

function publishPayload(dataStr) {
  console.log("[cmd send]", dataStr);
  logLine("ROS", `→ ${CONFIG.ros.webCommandTopic} | ${dataStr}`, "ros");

  if (!webCommandTopic || !isConnected) {
    logLine("SEND", "Not published — ROS not connected", "error");
    return;
  }

  webCommandTopic.publish({ data: dataStr });
  logLine("SEND", dataStr, "send");
}

function sendBuiltCommand() {
  const payload = buildCmdPayload();
  let dataStr = JSON.stringify(payload);

  if (payload.allLegs && payload.type === "move") {
    const { inner, outer, servo } = payload;
    cmdHistory.push(dataStr);
    cmdHistoryIdx = -1;
    for (const leg of CONFIG.legs) {
      const legPayload = { type: "move", leg_id: leg, inner, outer, servo };
      publishPayload(JSON.stringify(legPayload));
    }
  } else {
    cmdHistory.push(dataStr);
    cmdHistoryIdx = -1;
    publishPayload(dataStr);
  }

  const cmdInput = document.getElementById("cmd-f-command");
  if (cmdInput) {
    cmdInput.value = "";
    updateCmdPreview();
  }
}

function addSavedCommand() {
  const name = prompt("Name for this command:");
  if (!name) return;
  const nameTrim = name.trim();
  if (!nameTrim) return;

  const payload = buildCmdPayload();
  const json = JSON.stringify(payload);
  installSavedCommandChip(nameTrim, json);
}

function setupCommandForm() {
  const form = $("cmd-builder");
  const typeSelect = $("cmd-type");
  const legSelect = $("cmd-leg");

  typeSelect.addEventListener("change", buildCmdFields);
  legSelect.addEventListener("change", updateCmdPreview);

  form.addEventListener("submit", (e) => {
    e.preventDefault();
    sendBuiltCommand();
  });

  form.addEventListener("input", updateCmdPreview);

  $("cmd-save-btn").addEventListener("click", addSavedCommand);

  form.addEventListener("keydown", (e) => {
    const cmdInput = document.getElementById("cmd-f-command");
    if (!cmdInput || document.activeElement !== cmdInput) return;

    if (e.key === "ArrowUp") {
      e.preventDefault();
      if (cmdHistory.length === 0) return;
      if (cmdHistoryIdx === -1) {
        cmdHistoryDraft = cmdInput.value;
        cmdHistoryIdx = cmdHistory.length - 1;
      } else if (cmdHistoryIdx > 0) {
        cmdHistoryIdx--;
      }
      try {
        const parsed = JSON.parse(cmdHistory[cmdHistoryIdx]);
        cmdInput.value = parsed.command || cmdHistory[cmdHistoryIdx];
      } catch {
        cmdInput.value = cmdHistory[cmdHistoryIdx];
      }
      updateCmdPreview();
    } else if (e.key === "ArrowDown") {
      e.preventDefault();
      if (cmdHistoryIdx === -1) return;
      if (cmdHistoryIdx < cmdHistory.length - 1) {
        cmdHistoryIdx++;
        try {
          const parsed = JSON.parse(cmdHistory[cmdHistoryIdx]);
          cmdInput.value = parsed.command || cmdHistory[cmdHistoryIdx];
        } catch {
          cmdInput.value = cmdHistory[cmdHistoryIdx];
        }
      } else {
        cmdHistoryIdx = -1;
        cmdInput.value = cmdHistoryDraft;
      }
      updateCmdPreview();
    }
  });

  document.addEventListener("keydown", (e) => {
    if (!e.ctrlKey || e.shiftKey || e.altKey || e.metaKey) return;
    const num = parseInt(e.key, 10);
    if (isNaN(num) || num < 1) return;

    const panel = $("commands-panel");
    if (!panel || panel.hidden) return;

    const buttons = panel.querySelectorAll(".cmd-sticky-btn");
    if (num <= buttons.length) {
      e.preventDefault();
      buttons[num - 1].click();
    }
  });

  buildCmdFields();
  restoreSavedCommands();
}

const LEGS = [
  { id: "L0", label: "Leg 0", hasInner: false, hasOuter: true },
  { id: "L1", label: "Leg 1", hasInner: true, hasOuter: true },
  { id: "L2", label: "Leg 2", hasInner: true, hasOuter: true },
  { id: "L3", label: "Leg 3", hasInner: true, hasOuter: false },
];

function getLegValues(legId) {
  const inner = document.getElementById(`inner-${legId}`);
  const outer = document.getElementById(`outer-${legId}`);
  const servo = document.getElementById(`servo-${legId}`);

  return {
    innerVal: inner ? parseFloat(inner.value) || 0 : 0,
    outerVal: outer ? parseFloat(outer.value) || 0 : 0,
    servoVal: servo ? parseFloat(servo.value) || 0 : 0,
  };
}

/** Roll / pitch (deg) from per-leg `/lN/imu/roll_pitch_deg` — same as Gyro panel. */
function getImuRollPitchDegForLeg(legKey) {
  const r = legImuRegistry[legKey];
  if (!r) return { rollDeg: 0, pitchDeg: 0 };
  const rollDeg = Number.isFinite(r.rollDeg) ? r.rollDeg : 0;
  const pitchDeg = Number.isFinite(r.pitchDeg) ? r.pitchDeg : 0;
  return { rollDeg, pitchDeg };
}

/** @returns {Record<string, { id: string, inner_stepper: number, outer_stepper: number, hip: number, yaw: number, pitch: number, roll: number }>} */
function getLegPoseSnapshot() {
  const snapshot = {};
  for (const { id } of LEGS) {
    const key = id.toLowerCase();
    const base = getLegValues(id);
    const { rollDeg, pitchDeg } = getImuRollPitchDegForLeg(key);
    snapshot[key] = {
      id: key,
      inner_stepper: base.innerVal,
      outer_stepper: base.outerVal,
      hip: 0,
      yaw: 0,
      pitch: pitchDeg,
      roll: rollDeg,
    };
  }
  return snapshot;
}

function sendLegState(legId) {
  const { innerVal, outerVal, servoVal } = getLegValues(legId);
  const payload = { type: "move", leg_id: legId.toLowerCase(), inner: innerVal, outer: outerVal, servo: servoVal };
  const dataStr = JSON.stringify(payload);

  console.log("[sendLegState] isConnected:", isConnected,
    "| webCommandTopic:", webCommandTopic ? { name: webCommandTopic.name, type: webCommandTopic.messageType } : null,
    "| ros:", ros ? ros.isConnected : null);

  if (!webCommandTopic || !isConnected) {
    const reason = !webCommandTopic ? "topic not created" : "not connected";
    logLine("SEND", `${legId} not published — ${reason}`, "error");
    console.warn("[sendLegState] blocked:", reason);
    return;
  }

  const msg = { data: dataStr };

  console.log("[sendLegState] publishing to", webCommandTopic.name, ":", msg);
  logLine("ROS", `→ ${webCommandTopic.name} | ${dataStr}`, "ros");

  webCommandTopic.publish(msg);
  logLine("SEND", dataStr, "send");
}

function buildStepperGrid() {
  const grid = $("stepper-grid");
  grid.innerHTML = "";

  LEGS.forEach(({ id, label, hasInner, hasOuter }) => {
    const card = document.createElement("div");
    card.className = "leg-card";

    const headerRow = document.createElement("div");
    headerRow.className = "leg-heading-row";

    const title = document.createElement("span");
    title.className = "leg-heading";
    title.textContent = label;

    const enableBtn = document.createElement("button");
    enableBtn.type = "button";
    enableBtn.className = "leg-enable-status";
    enableBtn.dataset.legId = id;
    enableBtn.setAttribute("data-leg-enable-display", id);
    const idx = LEGS.findIndex((L) => L.id === id);
    const subscribeTopic = idx >= 0 ? legEnabledStateTopicPathForIndex(idx) : "";
    if (subscribeTopic) {
      enableBtn.dataset.rosTopic = subscribeTopic;
      enableBtn.title = `Listens to ${subscribeTopic}`;
    }
    enableBtn.textContent = "\u2014";
    enableBtn.setAttribute(
      "aria-label",
      `${id} enabled state${subscribeTopic ? ` from ${subscribeTopic}` : ""}; click to send enable toggle`
    );
    enableBtn.addEventListener("click", (e) => {
      e.stopPropagation();
      sendLegEnableToggleCommand(id);
    });

    headerRow.appendChild(title);
    headerRow.appendChild(enableBtn);
    card.appendChild(headerRow);

    if (hasInner) {
      card.appendChild(buildInputRow(id, "inner", "Inside"));
    }
    if (hasOuter) {
      card.appendChild(buildInputRow(id, "outer", "Outside"));
    }
    card.appendChild(buildInputRow(id, "servo", "Servo"));

    const setBtn = document.createElement("button");
    setBtn.type = "button";
    setBtn.className = "leg-set-btn";
    setBtn.textContent = "Set";
    setBtn.addEventListener("click", (e) => {
      e.stopPropagation();
      console.log("[Set clicked]", id);
      sendLegState(id);
    });
    card.appendChild(setBtn);

    grid.appendChild(card);
  });
}

function buildInputRow(legId, type, label, opts = {}) {
  const row = document.createElement("div");
  row.className = "stepper-row";

  const lbl = document.createElement("label");
  lbl.className = "stepper-label";
  lbl.setAttribute("for", `${type}-${legId}`);
  lbl.textContent = label;

  const input = document.createElement("input");
  input.type = "number";
  input.step =
    opts.step != null ? opts.step : type === "servo" ? "1" : "0.01";
  input.value = "0";
  input.id = `${type}-${legId}`;
  input.setAttribute("aria-label", `${legId} ${label}`);
  if (opts.title) input.title = opts.title;
  input.addEventListener("focus", () => input.select());

  input.addEventListener("keydown", (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      console.log("[Enter pressed]", legId, type, input.value);
      sendLegState(legId);
    }
  });

  row.appendChild(lbl);
  row.appendChild(input);
  return row;
}

function resetTestingState() {
  document.querySelectorAll("#stepper-grid input[type='number']").forEach((input) => {
    input.value = "0";
  });
  LEGS.forEach(({ id }) => sendLegState(id));
  logLine("TEST", "All legs reset");
}

function setupTestingControls() {
  buildStepperGrid();

  const resetBtn = $("reset-testing-btn");
  resetBtn.addEventListener("click", resetTestingState);
}

let panelZCounter = 26;

const UI_LAYOUT_STORAGE_KEY = "sam-ui-layout-v1";
let layoutSaveTimer = 0;

function scheduleSaveUILayout() {
  clearTimeout(layoutSaveTimer);
  layoutSaveTimer = window.setTimeout(() => {
    try {
      localStorage.setItem(UI_LAYOUT_STORAGE_KEY, JSON.stringify(buildUILayoutState()));
    } catch (_err) {
      // quota or private mode
    }
  }, 150);
}

/** @param {HTMLElement} panel */
function snapshotPanelLayout(panel) {
  return {
    hidden: panel.hidden,
    left: panel.style.left || "",
    top: panel.style.top || "",
    right: panel.style.right || "",
    bottom: panel.style.bottom || "",
    zIndex: panel.style.zIndex || "",
    placed: panel.dataset.placed === "1",
  };
}

function buildUILayoutState() {
  /** @type {Record<string, ReturnType<typeof snapshotPanelLayout>>} */
  const panels = {};
  document.querySelectorAll(".quick-panel").forEach((el) => {
    if (!(el instanceof HTMLElement) || !el.id) return;
    panels[el.id] = snapshotPanelLayout(el);
  });
  const layoutRadio = document.querySelector('input[name="robot-render-layout"]:checked');
  return {
    version: 1,
    savedAt: new Date().toISOString(),
    terminalPanelMoved,
    robotRenderLayout: layoutRadio?.value === "spread" ? "spread" : "parallel",
    panels,
  };
}

/** @param {HTMLElement} panel */
function clearPanelPositionStyles(panel) {
  panel.style.left = "";
  panel.style.top = "";
  panel.style.right = "";
  panel.style.bottom = "";
  panel.style.transform = "";
  panel.style.zIndex = "";
  delete panel.dataset.placed;
}

/**
 * @param {HTMLElement} panel
 * @param {Partial<ReturnType<typeof snapshotPanelLayout>> | null | undefined} snap
 */
function applyPanelSnapshotStyles(panel, snap) {
  if (!snap || typeof snap !== "object") return;
  panel.style.left = typeof snap.left === "string" ? snap.left : "";
  panel.style.top = typeof snap.top === "string" ? snap.top : "";
  panel.style.right = typeof snap.right === "string" ? snap.right : "";
  panel.style.bottom = typeof snap.bottom === "string" ? snap.bottom : "";
  panel.style.transform = "";
  if (typeof snap.zIndex === "string" && snap.zIndex) panel.style.zIndex = snap.zIndex;
  else panel.style.zIndex = "";
  if (snap.placed) panel.dataset.placed = "1";
  else delete panel.dataset.placed;
}

function syncLauncherButtonsToPanels() {
  document.querySelectorAll("[data-panel-target]").forEach((btn) => {
    const id = btn.getAttribute("data-panel-target");
    const panel = id ? $(id) : null;
    if (panel) btn.setAttribute("aria-expanded", String(!panel.hidden));
  });
}

function recomputePanelZCounterFromDom() {
  let maxZ = 26;
  document.querySelectorAll(".quick-panel").forEach((el) => {
    const z = Number.parseInt(el.style.zIndex, 10);
    if (!Number.isNaN(z)) maxZ = Math.max(maxZ, z);
  });
  panelZCounter = maxZ;
}

/**
 * @param {unknown} raw
 * @returns {raw is { version: number; panels: Record<string, ReturnType<typeof snapshotPanelLayout>>; terminalPanelMoved?: boolean; robotRenderLayout?: string }}
 */
function isValidUILayoutState(raw) {
  if (!raw || typeof raw !== "object") return false;
  const o = /** @type {Record<string, unknown>} */ (raw);
  if (o.version !== 1 || !o.panels || typeof o.panels !== "object") return false;
  return true;
}

/** @param {ReturnType<typeof buildUILayoutState>} state */
function applyUILayoutState(state) {
  if (!isValidUILayoutState(state)) return false;
  const wide = window.innerWidth > 1100;
  terminalPanelMoved = wide && !!state.terminalPanelMoved;

  for (const [id, snap] of Object.entries(state.panels)) {
    const panel = $(id);
    if (!(panel instanceof HTMLElement)) continue;
    if (!snap || typeof snap !== "object") continue;
    panel.hidden = !!snap.hidden;
    if (wide) applyPanelSnapshotStyles(panel, snap);
    else clearPanelPositionStyles(panel);
  }

  const parallel = document.querySelector('input[name="robot-render-layout"][value="parallel"]');
  const spread = document.querySelector('input[name="robot-render-layout"][value="spread"]');
  const wantSpread = state.robotRenderLayout === "spread";
  if (spread instanceof HTMLInputElement && parallel instanceof HTMLInputElement) {
    const currentlySpread = spread.checked;
    if (wantSpread !== currentlySpread) {
      if (wantSpread) {
        spread.checked = true;
        spread.dispatchEvent(new Event("change", { bubbles: true }));
      } else {
        parallel.checked = true;
        parallel.dispatchEvent(new Event("change", { bubbles: true }));
      }
    }
  }

  syncLauncherButtonsToPanels();
  recomputePanelZCounterFromDom();

  const termPanel = $("terminal-panel");
  if (wide && termPanel && !termPanel.hidden) {
    window.setTimeout(() => {
      getActiveTab()?.fitAddon?.fit();
      getActiveTab()?.term?.focus();
    }, 50);
  }
  return true;
}

function restoreUILayoutFromStorage() {
  try {
    const raw = localStorage.getItem(UI_LAYOUT_STORAGE_KEY);
    if (!raw) return;
    const parsed = JSON.parse(raw);
    applyUILayoutState(parsed);
  } catch (_err) {
    // ignore corrupt storage
  }
}

function resetUILayoutToBlankSlate() {
  document.querySelectorAll(".quick-panel").forEach((el) => {
    if (!(el instanceof HTMLElement)) return;
    el.hidden = true;
    clearPanelPositionStyles(el);
  });
  terminalPanelMoved = false;
  panelZCounter = 26;
  syncLauncherButtonsToPanels();

  const parallel = document.querySelector('input[name="robot-render-layout"][value="parallel"]');
  if (parallel instanceof HTMLInputElement) {
    parallel.checked = true;
    parallel.dispatchEvent(new Event("change", { bubbles: true }));
  }

  try {
    localStorage.removeItem(UI_LAYOUT_STORAGE_KEY);
  } catch (_err) {
    // ignore
  }
  scheduleSaveUILayout();
}

function downloadUILayoutFile() {
  const state = buildUILayoutState();
  const blob = new Blob([JSON.stringify(state, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
  a.href = url;
  a.download = `sam-ui-layout-${stamp}.json`;
  a.rel = "noopener";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function setupUILayoutPersistence() {
  document.querySelectorAll(".quick-panel").forEach((panel) => {
    const mo = new MutationObserver(() => scheduleSaveUILayout());
    mo.observe(panel, { attributes: true, attributeFilter: ["hidden"] });
  });

  document.querySelectorAll('input[name="robot-render-layout"]').forEach((radio) => {
    radio.addEventListener("change", () => scheduleSaveUILayout());
  });

  const exportBtn = $("ui-layout-export-btn");
  exportBtn?.addEventListener("click", () => {
    downloadUILayoutFile();
    logLine("INFO", "UI layout exported to file");
  });

  const importInput = $("ui-layout-import-input");
  const importBtn = $("ui-layout-import-btn");
  importBtn?.addEventListener("click", () => importInput?.click());
  importInput?.addEventListener("change", () => {
    const file = importInput.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const parsed = JSON.parse(String(reader.result || ""));
        if (!applyUILayoutState(parsed)) {
          logLine("ERROR", "Invalid layout file (expected version 1)");
          return;
        }
        try {
          localStorage.setItem(UI_LAYOUT_STORAGE_KEY, JSON.stringify(buildUILayoutState()));
        } catch (_err) {
          // still applied
        }
        logLine("INFO", "UI layout imported from file");
      } catch (_err) {
        logLine("ERROR", "Could not read layout file");
      }
      importInput.value = "";
    };
    reader.readAsText(file);
  });

  const resetBtn = $("ui-layout-reset-btn");
  resetBtn?.addEventListener("click", () => {
    if (
      !window.confirm(
        "Close all panels and clear saved positions? This cannot be undone except by re-arranging."
      )
    ) {
      return;
    }
    resetUILayoutToBlankSlate();
    logLine("INFO", "UI reset to blank slate (all panels closed)");
  });

  window.addEventListener("resize", () => {
    if (window.innerWidth > 1100) scheduleSaveUILayout();
  });
}

function bringPanelToFront(panel) {
  if (!(panel instanceof HTMLElement)) return;
  panelZCounter++;
  panel.style.zIndex = String(panelZCounter);
  scheduleSaveUILayout();
}

function placePanel(panel) {
  if (!(panel instanceof HTMLElement)) return;
  if (panel.id === "terminal-panel") return;
  if (panel.dataset.placed) return;
  if (window.innerWidth <= 1100) return;

  const idx = [...document.querySelectorAll(".quick-panel")].indexOf(panel);
  const offset = idx * 30;
  const right = 106 + offset;
  const top = 106 + offset;

  panel.style.right = `${right}px`;
  panel.style.top = `${top}px`;
  panel.style.left = "auto";
  panel.style.bottom = "auto";
}

function setupDraggablePanel(panel, handle) {
  if (!(panel instanceof HTMLElement) || !(handle instanceof HTMLElement)) return;

  let dragging = false;
  let startMouseX = 0;
  let startMouseY = 0;
  let dx = 0;
  let dy = 0;
  let rafId = 0;

  function applyTransform() {
    rafId = 0;
    panel.style.transform = `translate(${dx}px, ${dy}px)`;
  }

  function commitPosition() {
    const rect = panel.getBoundingClientRect();
    panel.style.transform = "";
    panel.style.left = `${rect.left}px`;
    panel.style.top = `${rect.top}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    panel.dataset.placed = "1";
  }

  handle.addEventListener("pointerdown", (e) => {
    if (window.innerWidth <= 1100) return;
    if (e.button !== 0) return;
    const tag = e.target.tagName;
    if (tag === "BUTTON" || tag === "INPUT" || tag === "SELECT" || tag === "A") return;

    bringPanelToFront(panel);
    startMouseX = e.clientX;
    startMouseY = e.clientY;
    dx = 0;
    dy = 0;

    dragging = true;
    document.body.classList.add("dragging-panel");
    handle.setPointerCapture(e.pointerId);
    e.preventDefault();
  });

  handle.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    dx = e.clientX - startMouseX;
    dy = e.clientY - startMouseY;
    if (!rafId) {
      rafId = requestAnimationFrame(applyTransform);
    }
  });

  function stopDragging(e) {
    if (!dragging) return;
    dragging = false;
    document.body.classList.remove("dragging-panel");
    if (rafId) {
      cancelAnimationFrame(rafId);
      rafId = 0;
    }
    if (dx !== 0 || dy !== 0) {
      commitPosition();
      scheduleSaveUILayout();
    }
    try {
      handle.releasePointerCapture(e.pointerId);
    } catch (_err) {
      // ignore
    }
  }

  handle.addEventListener("pointerup", stopDragging);
  handle.addEventListener("pointercancel", stopDragging);

  panel.addEventListener("pointerdown", () => bringPanelToFront(panel));
}

function placeConsolePanel(panel, force = false) {
  if (!(panel instanceof HTMLElement)) return;
  if (window.innerWidth <= 1100) {
    panel.style.left = "";
    panel.style.top = "";
    panel.style.right = "";
    return;
  }
  if (terminalPanelMoved && !force) return;

  const panelWidth = Math.min(980, Math.floor(window.innerWidth * 0.94 * 0.7));
  const panelHeight = Math.min(950, Math.floor(window.innerHeight * 0.88));
  const left = Math.max(12, Math.floor((window.innerWidth - panelWidth) / 2));
  const top = Math.max(12, Math.floor((window.innerHeight - panelHeight) / 2));

  panel.style.left = `${left}px`;
  panel.style.top = `${top}px`;
  panel.style.right = "auto";
  panel.style.bottom = "auto";
}

function setupDraggableConsolePanel() {
  const panel = $("terminal-panel");
  const handle = $("terminal-drag-handle");
  if (!(panel instanceof HTMLElement) || !(handle instanceof HTMLElement)) return;

  let dragging = false;
  let startMouseX = 0;
  let startMouseY = 0;
  let dx = 0;
  let dy = 0;
  let rafId = 0;

  function applyTransform() {
    rafId = 0;
    panel.style.transform = `translate(${dx}px, ${dy}px)`;
  }

  function commitPosition() {
    const rect = panel.getBoundingClientRect();
    panel.style.transform = "";
    panel.style.position = "fixed";
    panel.style.left = `${rect.left}px`;
    panel.style.top = `${rect.top}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    terminalPanelMoved = true;
  }

  handle.addEventListener("pointerdown", (e) => {
    if (window.innerWidth <= 1100) return;
    if (e.button !== 0) return;
    if (e.composedPath().includes($("terminal-settings-toggle-btn"))) {
      return;
    }

    bringPanelToFront(panel);
    startMouseX = e.clientX;
    startMouseY = e.clientY;
    dx = 0;
    dy = 0;

    dragging = true;
    document.body.classList.add("dragging-panel");
    handle.setPointerCapture(e.pointerId);
    e.preventDefault();
  });

  panel.addEventListener("pointerdown", () => bringPanelToFront(panel));

  handle.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    dx = e.clientX - startMouseX;
    dy = e.clientY - startMouseY;
    if (!rafId) {
      rafId = requestAnimationFrame(applyTransform);
    }
  });

  function stopDragging(e) {
    if (!dragging) return;
    dragging = false;
    document.body.classList.remove("dragging-panel");
    if (rafId) {
      cancelAnimationFrame(rafId);
      rafId = 0;
    }
    if (dx !== 0 || dy !== 0) {
      commitPosition();
      scheduleSaveUILayout();
    }
    try {
      handle.releasePointerCapture(e.pointerId);
    } catch (_err) {
      // ignore
    }
  }

  handle.addEventListener("pointerup", stopDragging);
  handle.addEventListener("pointercancel", stopDragging);

  window.addEventListener("resize", () => {
    if (!terminalPanelMoved) {
      placeConsolePanel(panel, true);
    }
  });
}

function setupPanelLauncher() {
  const launcherButtons = Array.from(
    document.querySelectorAll("[data-panel-target]")
  );
  const panels = launcherButtons
    .map((button) => $(button.getAttribute("data-panel-target")))
    .filter(Boolean);

  function isTypingContext(target) {
    if (!(target instanceof HTMLElement)) return false;
    const tag = target.tagName;
    return (
      tag === "INPUT" ||
      tag === "TEXTAREA" ||
      tag === "SELECT" ||
      target.isContentEditable
    );
  }

  launcherButtons.forEach((button) => {
    const targetId = button.getAttribute("data-panel-target");
    const panel = $(targetId);
    if (!panel) return;

    button.setAttribute("aria-expanded", "false");

    button.addEventListener("click", () => {
      const shouldOpen = panel.hidden;
      panel.hidden = !shouldOpen;
      if (shouldOpen && targetId === "terminal-panel") {
        placeConsolePanel(panel);
        setTimeout(() => {
          getActiveTab()?.fitAddon?.fit();
          getActiveTab()?.term?.focus();
        }, 50);
      }
      if (shouldOpen) {
        placePanel(panel);
        bringPanelToFront(panel);
      }
      button.setAttribute("aria-expanded", String(shouldOpen));
    });
  });

  // Numeric shortcuts: 1..N map top-to-bottom launcher buttons.
  document.addEventListener("keydown", (e) => {
    if (isTypingContext(e.target)) return;
    if (e.altKey || e.ctrlKey || e.metaKey || e.shiftKey) return;

    const index = Number.parseInt(e.key, 10);
    if (!Number.isInteger(index) || index < 1 || index > launcherButtons.length) {
      return;
    }

    const button = launcherButtons[index - 1];
    if (!button) return;
    e.preventDefault();
    button.click();
  });

  document.addEventListener("keydown", (e) => {
    if (e.key !== "Escape") return;
    if (document.fullscreenElement) return;
    // Close the topmost open panel
    const openPanels = panels
      .filter((p) => !p.hidden)
      .sort((a, b) => (parseInt(b.style.zIndex) || 0) - (parseInt(a.style.zIndex) || 0));
    if (openPanels.length > 0) {
      const top = openPanels[0];
      top.hidden = true;
      const btn = document.querySelector(`[data-panel-target="${top.id}"]`);
      if (btn) btn.setAttribute("aria-expanded", "false");
    }
  });

  const defaultButton = document.querySelector('[data-panel-target="commands-panel"]');
  if (defaultButton instanceof HTMLButtonElement) {
    defaultButton.setAttribute("aria-expanded", "false");
  }
}

function renderDeviceStatuses(devices) {
  const body = $("device-status-body");
  const countEl = $("device-count");
  if (!body) return;

  const sorted = [...devices].sort((a, b) => {
    const statusOrder = { connected: 0, unreachable: 1, disconnected: 2, unknown: 3 };
    const sa = statusOrder[a.status] ?? 3;
    const sb = statusOrder[b.status] ?? 3;
    if (sa !== sb) return sa - sb;
    return (a.name || a.ip || "").localeCompare(b.name || b.ip || "");
  });

  const json = JSON.stringify(sorted);
  if (json === lastDeviceJson) return;
  lastDeviceJson = json;

  body.innerHTML = "";

  if (sorted.length === 0) {
    const tr = document.createElement("tr");
    const td = document.createElement("td");
    td.colSpan = 3;
    td.className = "hint";
    td.style.textAlign = "center";
    td.style.padding = "16px 0";
    td.textContent = "No devices found on hotspot";
    tr.appendChild(td);
    body.appendChild(tr);
  } else {
    sorted.forEach((entry) => {
      const tr = document.createElement("tr");
      const status = String(entry.status || "unknown").toLowerCase();

      const nameTd = document.createElement("td");
      nameTd.textContent = entry.name || "-";

      const ipTd = document.createElement("td");
      ipTd.textContent = entry.ip || "-";

      const statusTd = document.createElement("td");
      statusTd.textContent = entry.status || "unknown";
      statusTd.className = `device-status-${status}`;

      tr.appendChild(nameTd);
      tr.appendChild(ipTd);
      tr.appendChild(statusTd);
      body.appendChild(tr);
    });
  }

  const connectedCount = devices.filter((d) => d.status === "connected").length;
  if (countEl) {
    countEl.textContent = `${connectedCount} connected, ${devices.length} total`;
  }
}

function getTerminalConnectionConfig() {
  const gatewayUrl = ($("terminal-gateway-url")?.value || "").trim();
  const host = ($("terminal-host")?.value || "").trim();
  const port = Number.parseInt(($("terminal-port")?.value || "").trim(), 10) || 22;
  const username = ($("terminal-user")?.value || "").trim();
  const password = $("terminal-password")?.value || "";

  return { gatewayUrl, host, port, username, password };
}

async function launchRosBridge() {
  const btn = $("launch-ros-bridge-btn");
  if (btn) {
    btn.disabled = true;
    btn.textContent = "Launching...";
  }

  const { gatewayUrl, host, port, username, password } = getTerminalConnectionConfig();
  if (!gatewayUrl || !host || !username) {
    logLine("ROS", "Terminal settings required to launch rosbridge", "error");
    if (btn) { btn.disabled = false; btn.textContent = "Launch"; }
    return;
  }

  try {
    await connectServiceGateway(gatewayUrl);
    serviceSocket.send(
      JSON.stringify({
        type: "run_command",
        host,
        port,
        username,
        password,
        command: "ros2 launch rosbridge_server rosbridge_websocket_launch.xml &",
      })
    );
    appendToTab(activeTabIndex, "\r\n$ ros2 launch rosbridge_server rosbridge_websocket_launch.xml &\r\n");
    logLine("ROS", "Launching rosbridge on " + host);
    setTimeout(() => {
      if (btn) { btn.disabled = false; btn.textContent = "Launch"; }
    }, 5000);
  } catch (err) {
    logLine("ROS", err.message || "Failed to launch rosbridge", "error");
    if (btn) { btn.disabled = false; btn.textContent = "Launch"; }
  }
}

async function requestDeviceScan() {
  if (deviceScanInFlight) return;

  const { gatewayUrl, host, port, username, password } = getTerminalConnectionConfig();
  if (!gatewayUrl || !host || !username || !password) {
    return;
  }

  deviceScanInFlight = true;
  try {
    await connectServiceGateway(gatewayUrl);
    serviceSocket.send(
      JSON.stringify({
        type: "scan_hotspot",
        host,
        port,
        username,
        password,
      })
    );
  } catch (err) {
    deviceScanInFlight = false;
    logLine("DEV", err.message || "Failed to request device scan", "error");
  }
}

function connectServiceGateway(gatewayUrl) {
  return new Promise((resolve, reject) => {
    if (
      serviceSocket &&
      serviceSocket.readyState === WebSocket.OPEN &&
      serviceSocketUrl === gatewayUrl
    ) {
      resolve();
      return;
    }

    if (serviceSocket && serviceSocket.readyState <= WebSocket.OPEN) {
      try {
        serviceSocket.close();
      } catch (_err) {
        // ignore
      }
    }

    serviceSocket = new WebSocket(gatewayUrl);
    serviceSocketUrl = gatewayUrl;

    serviceSocket.addEventListener("open", () => {
      updateTerminalStatusFromTabs();
      logLine("TERM", "Service gateway connected");
      resolve();
    });

    serviceSocket.addEventListener("close", () => {
      serviceSocket = null;
      serviceSocketUrl = "";
      updateTerminalStatusFromTabs();
      logLine("TERM", "Service gateway disconnected");
    });

    serviceSocket.addEventListener("error", () => {
      logLine("TERM", "Service gateway connection error", "error");
      reject(new Error("Terminal gateway connection error"));
    });

    serviceSocket.addEventListener("message", (event) => {
      let payload;
      try {
        payload = JSON.parse(event.data);
      } catch (_err) {
        return;
      }
      if (payload.type === "device_scan_result") {
        renderDeviceStatuses(payload.devices || []);
        deviceScanInFlight = false;
      } else if (payload.type === "device_scan_error") {
        logLine("DEV", payload.message || "Device scan failed", "error");
        deviceScanInFlight = false;
      }
    });
  });
}

function updateTerminalStatusFromTabs() {
  // Only show "Connected" once at least one tab has a usable SSH shell.
  const anyShellReady = terminalTabs.some((t) => t.shellReady);
  const serviceConnected = serviceSocket?.readyState === WebSocket.OPEN;
  if (anyShellReady) {
    setTerminalStatus("Connected", "connected");
  } else if (serviceConnected) {
    // Gateway up, but no SSH shell established yet.
    setTerminalStatus("Gateway", "disconnected");
  } else {
    setTerminalStatus("Disconnected", "disconnected");
  }
}

function installPanelCloseButtons() {
  document.querySelectorAll(".panel-drag-handle[data-drag-panel]").forEach((handle) => {
    const panelId = handle.getAttribute("data-drag-panel");
    if (!panelId) return;
    const panel = $(panelId);
    if (!panel) return;
    if (handle.querySelector(".panel-close-btn")) return;
    // Some panels already have an explicit close control (e.g. Render panel).
    if (handle.querySelector("#robot-render-close-btn")) return;

    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "panel-close-btn";
    btn.textContent = "×";
    btn.title = "Close";
    btn.setAttribute("aria-label", `Close ${panelId}`);
    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      panel.hidden = true;
      const launcherBtn = document.querySelector(`[data-panel-target="${panelId}"]`);
      if (launcherBtn) launcherBtn.setAttribute("aria-expanded", "false");
    });
    handle.appendChild(btn);
  });
}

function connectTabGateway(tab, gatewayUrl) {
  return new Promise((resolve, reject) => {
    if (tab.ws?.readyState === WebSocket.OPEN && tab.gatewayUrl === gatewayUrl) {
      resolve();
      return;
    }
    if (tab.ws?.readyState <= WebSocket.OPEN) {
      try {
        tab.ws?.close();
      } catch (_err) {}
    }

    const ws = new WebSocket(gatewayUrl);
    tab.ws = ws;
    tab.gatewayUrl = gatewayUrl;

    ws.addEventListener("open", () => {
      updateTerminalStatusFromTabs();
      resolve();
    });

    ws.addEventListener("close", () => {
      tab.shellReady = false;
      if (tab.ws === ws) tab.ws = null;
      updateTerminalStatusFromTabs();
    });

    ws.addEventListener("error", () => {
      const idx = terminalTabs.indexOf(tab);
      if (idx >= 0) appendToTab(idx, "\r\n[error] Could not connect to terminal gateway.\r\n");
      reject(new Error("Terminal gateway connection error"));
    });

    ws.addEventListener("message", (event) => {
      const idx = terminalTabs.indexOf(tab);
      if (idx < 0) return;
      let payload;
      try {
        payload = JSON.parse(event.data);
      } catch (_err) {
        appendToTab(idx, String(event.data) + "\r\n");
        return;
      }
      if (payload.type === "output") {
        appendToTab(idx, payload.data || "");
      } else if (payload.type === "status") {
        let message = String(payload.message || "");
        if (message.includes("Terminal gateway connected")) {
          message = "Terminal gateway connected (SSH not yet connected)";
        }
        appendToTab(idx, `\r\n[status] ${message}\r\n`);
        if (message.includes("Shell ready")) {
          tab.shellReady = true;
          updateTerminalStatusFromTabs();
        }
      } else if (payload.type === "error") {
        appendToTab(idx, `\r\n[error] ${payload.message}\r\n`);
        logLine("TERM", payload.message, "error");
      }
    });
  });
}

function createTerminalTab(label) {
  const tabsList = $("terminal-tabs-list");
  const tabsContent = $("terminal-tabs-content");
  if (!tabsList || !tabsContent) return null;

  const index = terminalTabs.length;
  const pane = document.createElement("div");
  pane.className = "terminal-tab-pane" + (index === 0 ? " is-active" : "");
  pane.setAttribute("role", "tabpanel");

  const container = document.createElement("div");
  container.className = "terminal-xterm-container";

  pane.appendChild(container);
  tabsContent.appendChild(pane);

  const term = new Terminal({
    cursorBlink: true,
    fontFamily: "ui-monospace, Menlo, Monaco, Consolas, monospace",
    fontSize: 13,
    theme: { background: "rgba(3, 8, 20, 0.99)", foreground: "#d1fae5" },
  });
  const fitAddon = new FitAddon();
  term.loadAddon(fitAddon);
  term.open(container);
  fitAddon.fit();

  const tab = {
    id: "tab-" + Date.now(),
    label,
    ws: null,
    gatewayUrl: "",
    term,
    fitAddon,
    shellReady: false,
    paneEl: pane,
    containerEl: container,
  };
  terminalTabs.push(tab);

  container.addEventListener("click", () => term.focus());

  term.onData((data) => {
    if (tab.ws?.readyState === WebSocket.OPEN) {
      tab.ws.send(JSON.stringify({ type: "input", data }));
      if (!tab.shellReady) {
        term.write(data);
        if (data === "\r" || data === "\n") {
          term.write("\r\n[Click Connect above to start an SSH session and run commands.]\r\n");
        }
      }
    } else {
      term.write(data);
      if (data === "\r" || data === "\n") {
        term.write("\r\n[Click Connect above to start an SSH session and run commands.]\r\n");
      }
    }
  });

  const tabBtn = document.createElement("button");
  tabBtn.type = "button";
  tabBtn.className = "terminal-tab-btn" + (index === 0 ? " is-active" : "");
  tabBtn.setAttribute("role", "tab");
  tabBtn.setAttribute("aria-selected", index === 0 ? "true" : "false");
  tabBtn.dataset.tabIndex = String(index);
  tabBtn.appendChild(document.createTextNode(label));
  tab.tabBtnEl = tabBtn;

  tabBtn.addEventListener("click", (e) => {
    if (e.target.classList.contains("tab-close")) return;
    switchTerminalTab(index);
  });

  const closeSpan = document.createElement("span");
  closeSpan.className = "tab-close";
  closeSpan.textContent = "×";
  closeSpan.setAttribute("aria-label", "Close tab");
  closeSpan.addEventListener("click", (e) => {
    e.stopPropagation();
    closeTerminalTab(index);
  });
  tabBtn.appendChild(closeSpan);

  tabsList.appendChild(tabBtn);

  const resizeObserver = new ResizeObserver(() => {
    if (terminalTabs[index] && activeTabIndex === index) {
      terminalTabs[index].fitAddon?.fit();
    }
  });
  resizeObserver.observe(container);

  return tab;
}

function switchTerminalTab(index) {
  if (index < 0 || index >= terminalTabs.length) return;
  activeTabIndex = index;
  terminalTabs.forEach((tab, i) => {
    const isActive = i === index;
    tab.paneEl.classList.toggle("is-active", isActive);
    tab.tabBtnEl?.classList.toggle("is-active", isActive);
    tab.tabBtnEl?.setAttribute("aria-selected", isActive ? "true" : "false");
  });
  const tab = terminalTabs[index];
  tab?.fitAddon?.fit();
  tab?.term?.focus();
}

function closeTerminalTab(index) {
  if (terminalTabs.length <= 1) return;
  const tab = terminalTabs[index];
  if (tab?.ws) {
    try {
      tab.ws.close();
    } catch (_err) {}
  }
  tab?.term?.dispose();
  tab?.paneEl?.remove();
  tab?.tabBtnEl?.remove();
  terminalTabs.splice(index, 1);
  if (activeTabIndex >= terminalTabs.length) activeTabIndex = Math.max(0, terminalTabs.length - 1);
  if (activeTabIndex > index) activeTabIndex--;
  terminalTabs.forEach((t, i) => {
    t.tabBtnEl.dataset.tabIndex = String(i);
  });
  switchTerminalTab(activeTabIndex);
}

function connectActiveTab() {
  const tab = getActiveTab();
  if (!tab) return;
  const { gatewayUrl, host, port, username, password } = getTerminalConnectionConfig();
  if (!gatewayUrl || !host || !username) {
    logLine("TERM", "Gateway, host, and user are required", "error");
    return;
  }
  const connectBtn = $("terminal-connect-btn");
  connectBtn.disabled = true;
  connectTabGateway(tab, gatewayUrl)
    .then(() => {
      tab.ws.send(
        JSON.stringify({ type: "connect", host, port, username, password })
      );
      appendToTab(terminalTabs.indexOf(tab), "\r\n[Connecting to " + host + "...]\r\n");
      logLine("TERM", "Connecting to " + host);
    })
    .catch((err) => {
      logLine("TERM", err.message || "Connection failed", "error");
    })
    .finally(() => {
      connectBtn.disabled = false;
    });
}

function setupTerminalForm() {
  const connectBtn = $("terminal-connect-btn");
  const settingsBtn = $("terminal-settings-toggle-btn");
  const settingsPanel = $("terminal-settings");
  const gatewayInput = $("terminal-gateway-url");
  const passwordInput = $("terminal-password");
  const refreshDevicesBtn = $("refresh-devices-btn");
  const autoRefreshDevices = $("auto-refresh-devices");

  gatewayInput.value = CONFIG.terminal.gatewayUrl;
  if (passwordInput && !passwordInput.value) {
    passwordInput.value = DEFAULT_TERMINAL_PASSWORD;
  }

  if (settingsBtn && settingsPanel) {
    function syncSettingsToggleState() {
      const expanded = !settingsPanel.hidden;
      settingsBtn.setAttribute("aria-expanded", String(expanded));
      settingsBtn.classList.toggle("is-active", expanded);
    }
    syncSettingsToggleState();
    settingsBtn.addEventListener("click", () => {
      settingsPanel.hidden = !settingsPanel.hidden;
      syncSettingsToggleState();
    });
  }

  if (refreshDevicesBtn) {
    refreshDevicesBtn.addEventListener("click", () => requestDeviceScan());
  }
  if (autoRefreshDevices) {
    autoRefreshDevices.addEventListener("change", () => {
      if (deviceRefreshTimer) {
        clearInterval(deviceRefreshTimer);
        deviceRefreshTimer = null;
      }
      if (autoRefreshDevices.checked) {
        requestDeviceScan();
        deviceRefreshTimer = setInterval(requestDeviceScan, 5000);
      }
    });
    if (autoRefreshDevices.checked) {
      deviceRefreshTimer = setInterval(requestDeviceScan, 5000);
    }
  }

  createTerminalTab("1");
  let tabCounter = 2;

  $("terminal-add-tab-btn")?.addEventListener("click", () => {
    const tab = createTerminalTab(String(tabCounter++));
    if (tab) switchTerminalTab(terminalTabs.length - 1);
  });

  connectBtn?.addEventListener("click", connectActiveTab);

  const tabsContent = $("terminal-tabs-content");
  if (tabsContent) {
    const ro = new ResizeObserver(() => {
      getActiveTab()?.fitAddon?.fit();
    });
    ro.observe(tabsContent);
  }

  appendToTab(0, "S.A.M. Jetson terminal ready.\r\n");

  requestDeviceScan();

  setTimeout(() => {
    const cfg = getTerminalConnectionConfig();
    if (cfg.gatewayUrl && cfg.host && cfg.username) {
      connectActiveTab();
    }
  }, 300);
}

function setupInitialFocus() {
  const launcherButton = document.querySelector('[data-panel-target="commands-panel"]');
  if (launcherButton instanceof HTMLButtonElement) {
    launcherButton.focus();
  }
}

function setupTrajectoryPanel() {
  const btn = $("trajectory-toggle-pose-sequence-btn");
  const block = $("trajectory-sequence-block");
  if (!btn || !block) return;

  btn.addEventListener("click", () => {
    const opening = block.hidden;
    block.hidden = !opening;
    btn.setAttribute("aria-expanded", String(opening));
    btn.textContent = opening
      ? "Hide global pose sequence"
      : "Show global pose sequence";
  });
}

window.addEventListener("DOMContentLoaded", () => {
  setupRos();
  setupDraggableConsolePanel();
  setupPanelLauncher();
  setupCommandForm();
  setupTestingControls();
  initRobotRenderUI({
    panel: $("robot-render-panel"),
    getPose: getLegPoseSnapshot,
  });
  setupTerminalForm();
  setupTrajectoryPanel();
  installPanelCloseButtons();
  const gyroZeroRollPitchBtn = $("gyro-zero-roll-pitch-btn");
  if (gyroZeroRollPitchBtn) {
    gyroZeroRollPitchBtn.addEventListener("click", zeroImuRollPitchDisplay);
  }
  setupInitialFocus();

  // Make all panels with drag handles draggable
  document.querySelectorAll(".panel-drag-handle[data-drag-panel]").forEach((handle) => {
    const panel = $(handle.dataset.dragPanel);
    if (panel) setupDraggablePanel(panel, handle);
  });

  // Log filter buttons
  document.querySelectorAll(".log-filter-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      activeLogFilter = btn.dataset.logFilter || "all";
      document.querySelectorAll(".log-filter-btn").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      const lines = document.querySelectorAll("#log-output .log-line");
      lines.forEach((line) => {
        if (activeLogFilter === "all" || line.dataset.logKind === activeLogFilter) {
          line.style.display = "";
        } else {
          line.style.display = "none";
        }
      });
    });
  });

  const launchBtn = $("launch-ros-bridge-btn");
  if (launchBtn) {
    launchBtn.addEventListener("click", launchRosBridge);
  }

  const E_STOP_PAYLOAD = JSON.stringify({ type: "estop", value: true });
  const START_PAYLOAD = JSON.stringify({ type: "estop", value: false });
  const estopBtn = $("estop-btn");
  const startBtn = $("start-btn");
  if (estopBtn) {
    estopBtn.addEventListener("click", () => {
      publishPayload(E_STOP_PAYLOAD);
      startBtn?.removeAttribute("hidden");
    });
  }
  if (startBtn) {
    startBtn.addEventListener("click", () => {
      publishPayload(START_PAYLOAD);
      startBtn.hidden = true;
    });
  }
  document.addEventListener(
    "keydown",
    (e) => {
      if (e.key !== "`" && e.key !== "Backquote") return;
      if (e.repeat) return;
      e.preventDefault();
      e.stopPropagation();
      publishPayload(E_STOP_PAYLOAD);
      startBtn?.removeAttribute("hidden");
      estopBtn?.focus();
    },
    true
  );

  restoreUILayoutFromStorage();

  logLine("INFO", "S.A.M. Control Interface ready");
  logLine(
    "HINT",
    "Keyboard hints: ` = E-Stop (always, including over inputs and terminal), Tab to move, Space/Enter to activate, Alt+S = command preset, Alt+C = command line. In stepper fields, Enter sends velocity."
  );

  setupUILayoutPersistence();
});