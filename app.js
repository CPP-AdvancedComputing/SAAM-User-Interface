import * as ROSLIB from "roslib";
import { Terminal } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import "@xterm/xterm/css/xterm.css";

// Basic config for ROS topics and legs/joints
const CONFIG = {
  ros: {
    bridgeUrl: "ws://10.12.64.222:9090",
    commandTopic: "/sam/command",
    testingTopic: "/sam/testing_state",
    webCommandTopic: "/web/command",
    commandMessageType: "std_msgs/msg/String",
    testingMessageType: "std_msgs/msg/String",
  },
  terminal: {
    gatewayUrl: "ws://localhost:8787",
  },
  legs: ["l0", "l1", "l2", "l3"],
  joints: ["inner_stepper", "outer_stepper", "hip", "yaw", "pitch", "roll"],
};
const DEFAULT_TERMINAL_PASSWORD = import.meta.env.VITE_TERMINAL_PASSWORD || "";

let ros = null;
let commandTopic = null;
let testingTopic = null;
let webCommandTopic = null;
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

function cleanupRosState() {
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
  const rosMenuBtn = $("ros-menu-btn");
  const rosMetaPanel = $("ros-meta-panel");
  if (endpoint) {
    endpoint.textContent = CONFIG.ros.bridgeUrl;
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
}


function getCurrentMode() {
  const radios = document.querySelectorAll('input[name="mode"]');
  for (const r of radios) {
    if (r.checked) return r.value;
  }
  return "enabled";
}

function setupModeToggle() {
  const radios = document.querySelectorAll('input[name="mode"]');

  radios.forEach((r) => {
    r.addEventListener("change", () => {
      const mode = getCurrentMode();
      logLine("MODE", "Changed mode to " + mode);
      sendModeChange(mode);
    });
  });
}

function sendModeChange(mode) {
  if (!commandTopic || !isConnected) {
    logLine("SEND", "Cannot send mode change, not connected", "error");
    return;
  }

  const payload = {
    type: "mode_change",
    mode,
  };

  commandTopic.publish({ data: JSON.stringify(payload) });
  logLine("SEND", "Mode change sent: " + JSON.stringify(payload), "send");
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

  const payload = buildCmdPayload();
  const json = JSON.stringify(payload);
  const container = $("cmd-saved");

  const wrapper = document.createElement("div");
  wrapper.className = "cmd-sticky";

  const btn = document.createElement("button");
  btn.type = "button";
  btn.className = "cmd-sticky-btn";
  btn.textContent = name;
  btn.dataset.cmd = json;

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
  removeBtn.addEventListener("click", () => wrapper.remove());

  wrapper.appendChild(btn);
  wrapper.appendChild(removeBtn);
  container.appendChild(wrapper);
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

    const heading = document.createElement("div");
    heading.className = "leg-heading";
    heading.textContent = label;
    card.appendChild(heading);

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

function buildInputRow(legId, type, label) {
  const row = document.createElement("div");
  row.className = "stepper-row";

  const lbl = document.createElement("label");
  lbl.className = "stepper-label";
  lbl.setAttribute("for", `${type}-${legId}`);
  lbl.textContent = label;

  const input = document.createElement("input");
  input.type = "number";
  input.step = type === "servo" ? "1" : "0.01";
  input.value = "0";
  input.id = `${type}-${legId}`;
  input.setAttribute("aria-label", `${legId} ${label}`);
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

function bringPanelToFront(panel) {
  if (!(panel instanceof HTMLElement)) return;
  panelZCounter++;
  panel.style.zIndex = String(panelZCounter);
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

  const panelWidth = Math.min(1400, Math.floor(window.innerWidth * 0.94));
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
  const anyConnected = terminalTabs.some((t) => t.ws?.readyState === WebSocket.OPEN);
  const serviceConnected = serviceSocket?.readyState === WebSocket.OPEN;
  setTerminalStatus(anyConnected || serviceConnected ? "Connected" : "Disconnected", anyConnected || serviceConnected ? "connected" : "disconnected");
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
        appendToTab(idx, `\r\n[status] ${payload.message}\r\n`);
        if (String(payload.message || "").includes("Shell ready")) {
          tab.shellReady = true;
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

window.addEventListener("DOMContentLoaded", () => {
  setupRos();
  setupModeToggle();
  setupDraggableConsolePanel();
  setupPanelLauncher();
  setupCommandForm();
  setupTestingControls();
  setupTerminalForm();
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
  document.addEventListener("keydown", (e) => {
    if (e.key !== "`" && e.key !== "Backquote") return;
    const tag = e.target?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || e.target?.isContentEditable) return;
    e.preventDefault();
    publishPayload(E_STOP_PAYLOAD);
    estopBtn?.focus();
  });

  logLine("INFO", "S.A.M. Control Interface ready");
  logLine(
    "HINT",
    "Keyboard hints: ` = E-Stop, Tab to move, Space/Enter to activate, Alt+S = command preset, Alt+C = command line. In stepper fields, Enter sends velocity."
  );
});