import * as ROSLIB from "roslib";

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
let terminalSocket = null;
let terminalSocketUrl = "";
let terminalHistory = [];
let terminalHistoryIndex = -1;
let terminalPromptHidden = false;
let terminalCurrentCwd = "~";
let terminalCurrentUser = "sam";
let terminalCurrentHost = "10.12.64.222";
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

function appendTerminalOutput(message) {
  const output = $("terminal-output");
  if (!output) return;
  output.textContent += message;
  requestAnimationFrame(() => {
    output.scrollTop = output.scrollHeight;
  });
}

function formatPromptPath(cwd, username) {
  const homePrefix = `/home/${username}`;
  if (!cwd) return "~";
  if (cwd === homePrefix) return "~";
  if (cwd.startsWith(homePrefix + "/")) {
    return "~" + cwd.slice(homePrefix.length);
  }
  return cwd;
}

function setTerminalPrompt(username, host, cwd) {
  const prompt = $("terminal-prompt");
  if (!prompt) return;
  terminalCurrentUser = username || terminalCurrentUser;
  terminalCurrentHost = host || terminalCurrentHost;
  terminalCurrentCwd = formatPromptPath(cwd || terminalCurrentCwd, terminalCurrentUser);
  prompt.textContent = `${terminalCurrentUser}@${terminalCurrentHost}:${terminalCurrentCwd}$`;
}

function setTerminalPromptVisible(visible) {
  terminalPromptHidden = !visible;
  const form = $("terminal-shell-form");
  const hint = $("terminal-ctrl-hint");
  const cmdInput = $("terminal-command");
  const termWindow = $("terminal-window");
  if (form) form.hidden = !visible;
  if (hint) hint.hidden = visible;
  if (termWindow) termWindow.classList.toggle("terminal-listening", !visible);
  if (cmdInput) cmdInput.disabled = !visible;
  if (visible) {
    if (cmdInput) cmdInput.focus();
  } else {
    const output = $("terminal-output");
    if (output) output.focus();
  }
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
  const leg = $("cmd-leg").value;

  if (type === "walk") {
    const el = document.getElementById("cmd-f-distance");
    const distance = el ? String(el.value || "").trim() || "5" : "5";
    return { type: "walk", distance };
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
  const dataStr = JSON.stringify(payload);

  cmdHistory.push(dataStr);
  cmdHistoryIdx = -1;

  publishPayload(dataStr);

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
    try {
      JSON.parse(btn.dataset.cmd);
    } catch {
      logLine("CMD", "Invalid JSON in saved command", "error");
      return;
    }
    publishPayload(btn.dataset.cmd);
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

  const panelWidth = Math.min(860, Math.floor(window.innerWidth * 0.72));
  const left = Math.max(24, Math.floor((window.innerWidth - panelWidth) / 2 + 30));
  const top = Math.max(88, Math.floor((window.innerHeight - 560) / 2));

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
    await connectTerminalGateway(gatewayUrl);
    terminalSocket.send(
      JSON.stringify({
        type: "run_command",
        host,
        port,
        username,
        password,
        command: "ros2 launch rosbridge_server rosbridge_websocket_launch.xml &",
      })
    );
    appendTerminalOutput(
      `\n${terminalCurrentUser}@${terminalCurrentHost}:${terminalCurrentCwd}$ ros2 launch rosbridge_server rosbridge_websocket_launch.xml &\n`
    );
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
    await connectTerminalGateway(gatewayUrl);
    terminalSocket.send(
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

function connectTerminalGateway(gatewayUrl) {
  return new Promise((resolve, reject) => {
    if (
      terminalSocket &&
      terminalSocket.readyState === WebSocket.OPEN &&
      terminalSocketUrl === gatewayUrl
    ) {
      resolve();
      return;
    }

    if (terminalSocket && terminalSocket.readyState <= WebSocket.OPEN) {
      try {
        terminalSocket.close();
      } catch (_err) {
        // ignore close errors
      }
    }

    terminalSocket = new WebSocket(gatewayUrl);
    terminalSocketUrl = gatewayUrl;

    terminalSocket.addEventListener("open", () => {
      setTerminalStatus("Connected", "connected");
      logLine("TERM", "Connected to terminal gateway");
      resolve();
    });

    terminalSocket.addEventListener("close", () => {
      setTerminalStatus("Disconnected", "disconnected");
      logLine("TERM", "Terminal gateway disconnected");
      terminalSocket = null;
      terminalSocketUrl = "";
    });

    terminalSocket.addEventListener("error", () => {
      setTerminalStatus("Error", "error");
      logLine("TERM", "Terminal gateway connection error", "error");
      appendTerminalOutput("\n[error] Could not connect to terminal gateway.\n");
      reject(new Error("Terminal gateway connection error"));
    });

    terminalSocket.addEventListener("message", (event) => {
      let payload;
      try {
        payload = JSON.parse(event.data);
      } catch (_err) {
        appendTerminalOutput(String(event.data) + "\n");
        return;
      }

      if (payload.type === "output") {
        appendTerminalOutput(payload.data || "");
      } else if (payload.type === "status") {
        appendTerminalOutput(`\n[status] ${payload.message}\n`);
      } else if (payload.type === "exit") {
        if (payload.cwd) {
          setTerminalPrompt(terminalCurrentUser, terminalCurrentHost, payload.cwd);
        }
        setTerminalPromptVisible(true);
      } else if (payload.type === "device_scan_result") {
        renderDeviceStatuses(payload.devices || []);
        deviceScanInFlight = false;
      } else if (payload.type === "device_scan_error") {
        logLine("DEV", payload.message || "Device scan failed", "error");
        deviceScanInFlight = false;
      } else if (payload.type === "error") {
        const msg = String(payload.message || "");
        if (msg.toLowerCase().includes("unsupported message type")) {
          deviceScanInFlight = false;
          return;
        }
        appendTerminalOutput(`\n[error] ${payload.message}\n`);
        logLine("TERM", payload.message, "error");
        deviceScanInFlight = false;
        setTerminalPromptVisible(true);
      }
    });
  });
}

function setupTerminalForm() {
  const form = $("terminal-shell-form");
  const settingsBtn = $("terminal-settings-toggle-btn");
  const settingsPanel = $("terminal-settings");
  const gatewayInput = $("terminal-gateway-url");
  const hostInput = $("terminal-host");
  const portInput = $("terminal-port");
  const userInput = $("terminal-user");
  const passwordInput = $("terminal-password");
  const commandInput = $("terminal-command");
  const refreshDevicesBtn = $("refresh-devices-btn");
  const autoRefreshDevices = $("auto-refresh-devices");

  if (!form) return;

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

  setTerminalPrompt(
    (userInput.value || "sam").trim(),
    (hostInput.value || "10.42.0.1").trim(),
    "~"
  );
  appendTerminalOutput("S.A.M. Jetson terminal ready.\n");

  const termWindow = $("terminal-window");
  const termPanel = $("terminal-panel");
  if (termWindow) {
    termWindow.addEventListener("click", (e) => {
      if (e.target === termWindow || e.target === $("terminal-output")) {
        (terminalPromptHidden ? $("terminal-output") : commandInput)?.focus();
      }
    });
  }

  document.addEventListener("keydown", (e) => {
    if (!terminalPromptHidden || !termPanel || termPanel.hidden) return;
    if (e.ctrlKey && e.key === "c") {
      e.preventDefault();
      if (terminalSocket && terminalSocket.readyState === WebSocket.OPEN) {
        terminalSocket.send(JSON.stringify({ type: "interrupt" }));
        setTerminalPromptVisible(true);
      }
    }
  });

  if (refreshDevicesBtn) {
    refreshDevicesBtn.addEventListener("click", () => {
      requestDeviceScan();
    });
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

  commandInput.addEventListener("keydown", (e) => {
    if (e.key === "ArrowUp") {
      if (terminalHistory.length === 0) return;
      e.preventDefault();
      terminalHistoryIndex = Math.max(0, terminalHistoryIndex - 1);
      commandInput.value = terminalHistory[terminalHistoryIndex];
      return;
    }

    if (e.key === "ArrowDown") {
      if (terminalHistory.length === 0) return;
      e.preventDefault();
      terminalHistoryIndex = Math.min(
        terminalHistory.length,
        terminalHistoryIndex + 1
      );
      commandInput.value =
        terminalHistoryIndex === terminalHistory.length
          ? ""
          : terminalHistory[terminalHistoryIndex];
    }
  });

  form.addEventListener("submit", async (e) => {
    e.preventDefault();
    const gatewayUrl = (gatewayInput.value || "").trim();
    const host = (hostInput.value || "").trim();
    const port = Number.parseInt((portInput.value || "").trim(), 10) || 22;
    const username = (userInput.value || "").trim();
    const password = passwordInput.value || "";
    const command = (commandInput.value || "").trim();

    if (!gatewayUrl || !host || !username || !command) {
      logLine(
        "TERM",
        "Gateway URL, host, user, and terminal command are required",
        "error"
      );
      return;
    }

    terminalHistory.push(command);
    terminalHistoryIndex = terminalHistory.length;
    appendTerminalOutput(
      `\n${terminalCurrentUser}@${terminalCurrentHost}:${terminalCurrentCwd}$ ${command}\n`
    );
    commandInput.value = "";
    setTerminalPrompt(username, host, terminalCurrentCwd);
    setTerminalPromptVisible(false);

    try {
      await connectTerminalGateway(gatewayUrl);
      terminalSocket.send(
        JSON.stringify({
          type: "run_command",
          host,
          port,
          username,
          password,
          command,
        })
      );
      logLine("TERM", `Command sent to ${host}: ${command}`);
    } catch (err) {
      appendTerminalOutput(`\n[error] ${err.message || "Failed to send terminal command"}\n`);
      logLine("TERM", err.message || "Failed to send terminal command", "error");
      setTerminalPromptVisible(true);
    }
  });

  requestDeviceScan();
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


  logLine("INFO", "S.A.M. Control Interface ready");
  logLine(
    "HINT",
    "Keyboard hints: Tab to move, Space/Enter to activate, Alt+S = command preset, Alt+C = command line. In stepper fields, Enter sends velocity."
  );
});