const fs = require("fs");
const path = require("path");
const os = require("os");
const { spawn } = require("child_process");
const { WebSocketServer } = require("ws");
const { Client } = require("ssh2");

const PORT = Number(process.env.TERMINAL_GATEWAY_PORT || 8787);
/**
 * sudo reads passwords via open("/dev/tty"), not only isatty(stdin). Some SSH PTY sessions
 * still fail that open while `tty` and `test -t 0` look fine. Wrapping in `script(1)` fixes it.
 * Set SAM_SKIP_SCRIPT_WRAPPER=1 to disable (bsdutils / util-linux provides `script` on Jetson/Ubuntu).
 */
const SKIP_SCRIPT_WRAPPER = process.env.SAM_SKIP_SCRIPT_WRAPPER === "1";
const UI_LAYOUT_STORE_PATH = process.env.SAM_UI_LAYOUT_STORE_PATH
  ? path.resolve(process.env.SAM_UI_LAYOUT_STORE_PATH)
  : path.resolve(__dirname, "sam-ui-layout-store.json");
const WALK_PRESETS_STORE_PATH = process.env.SAM_WALK_PRESETS_STORE_PATH
  ? path.resolve(process.env.SAM_WALK_PRESETS_STORE_PATH)
  : path.resolve(__dirname, "sam-walk-presets-store.json");
const PICO_WEBREPL_CLIENT_PATH = path.resolve(__dirname, "scripts", "pico_webrepl_client.py");
let nextCommandId = 1;

// Persistent set of known hotspot IPs (survives across scans)
const knownHotspotIPs = new Set();

// IPs to never show in device list
const blockedIPs = new Set(["10.42.0.106"]);

function isValidUILayoutState(raw) {
  if (!raw || typeof raw !== "object") return false;
  if (raw.version !== 1) return false;
  if (!raw.panels || typeof raw.panels !== "object") return false;
  return true;
}

function loadSharedUILayoutState() {
  try {
    if (!fs.existsSync(UI_LAYOUT_STORE_PATH)) return null;
    const raw = fs.readFileSync(UI_LAYOUT_STORE_PATH, "utf8");
    const parsed = JSON.parse(raw);
    if (!isValidUILayoutState(parsed)) return null;
    return parsed;
  } catch (_err) {
    return null;
  }
}

function persistSharedUILayoutState(state) {
  if (!isValidUILayoutState(state)) return false;
  try {
    fs.mkdirSync(path.dirname(UI_LAYOUT_STORE_PATH), { recursive: true });
    fs.writeFileSync(UI_LAYOUT_STORE_PATH, JSON.stringify(state), "utf8");
    return true;
  } catch (_err) {
    return false;
  }
}

let sharedUILayoutState = loadSharedUILayoutState();

function isValidWalkPresetEntry(raw) {
  if (!raw || typeof raw !== "object") return false;
  if (typeof raw.name !== "string" || !raw.name.trim()) return false;
  if (!Array.isArray(raw.sequence) || raw.sequence.length === 0) return false;
  return true;
}

function normalizeWalkPresets(raw) {
  if (!Array.isArray(raw)) return [];
  return raw.filter(isValidWalkPresetEntry).map((entry) => ({
    name: String(entry.name).trim(),
    count: String(entry.count ?? "1"),
    sequence: entry.sequence,
    savedAt:
      typeof entry.savedAt === "string" && entry.savedAt
        ? entry.savedAt
        : new Date().toISOString(),
  }));
}

function loadSharedWalkPresets() {
  try {
    if (!fs.existsSync(WALK_PRESETS_STORE_PATH)) return [];
    const raw = fs.readFileSync(WALK_PRESETS_STORE_PATH, "utf8");
    return normalizeWalkPresets(JSON.parse(raw));
  } catch (_err) {
    return [];
  }
}

function persistSharedWalkPresets(presets) {
  const normalized = normalizeWalkPresets(presets);
  try {
    fs.mkdirSync(path.dirname(WALK_PRESETS_STORE_PATH), { recursive: true });
    fs.writeFileSync(WALK_PRESETS_STORE_PATH, JSON.stringify(normalized), "utf8");
    console.log(`[walk-presets] wrote ${WALK_PRESETS_STORE_PATH} (${normalized.length} preset(s))`);
    return true;
  } catch (err) {
    console.error("[walk-presets] write failed:", err && err.message ? err.message : err);
    return false;
  }
}

let sharedWalkPresets = loadSharedWalkPresets();

function stripAnsi(input) {
  return input
    .replace(/\u001b\][^\u0007]*\u0007/g, "")
    .replace(/\u001b\[[0-9;?]*[ -/]*[@-~]/g, "")
    .replace(/\r/g, "");
}

function extractExitMarker(buffer) {
  const match = buffer.match(/__SAM_EXIT__:(\d+):(-?\d+):([^\n]*)(\n|$)/);
  if (!match) return { before: buffer, marker: null, after: "" };
  const idx = match.index;
  const before = buffer.slice(0, idx);
  const after = buffer.slice(idx + match[0].length);
  return {
    before,
    marker: { id: Number(match[1]), code: Number(match[2]), cwd: (match[3] || "").trim() },
    after,
  };
}

function send(ws, payload) {
  if (ws.readyState === 1) {
    ws.send(JSON.stringify(payload));
  }
}

function toCrlf(str) {
  return str.replace(/\r?\n/g, "\r\n");
}

/**
 * Stop the foreground process like a real interactive terminal by sending the PTY interrupt
 * character. Avoid SSH channel signals here because some servers apply them to the shell
 * session itself, which drops the connection instead of only interrupting the current job.
 */
function sendPtyInterrupt(stream) {
  if (!stream) return;
  try {
    stream.write("\x03");
  } catch (_err) {
    // ignore
  }
}


/**
 * Stop foreground process like a real terminal: intr byte on PTY, plus SSH SIGNAL when the
 * kernel line discipline is wrong (does not close the SSH session).
 */
function forwardSigintToShell(stream) {
  if (!stream) return;
  try {
    stream.write("\x03");
  } catch (_err) {
    // ignore
  }
  try {
    stream.signal("INT");
  } catch (_err) {
    // ignore — not all servers implement channel signals
  }
}

const wss = new WebSocketServer({ port: PORT });

wss.on("connection", (ws) => {
  const session = {
    ssh: null,
    shell: null,
    lineBuffer: "",
    queue: [],
    active: null,
    connectionKey: "",
    ready: false,
  };

  send(ws, { type: "status", message: "Terminal gateway connected." });

  function resetShellState() {
    session.shell = null;
    session.ssh = null;
    session.connectionKey = "";
    session.queue = [];
    session.active = null;
    session.lineBuffer = "";
    session.ready = false;
  }

  function closeShellSession() {
    if (session.shell) {
      try {
        session.shell.end();
      } catch (_err) {
        // ignore
      }
    }
    if (session.ssh) {
      try {
        session.ssh.end();
      } catch (_err) {
        // ignore
      }
    }
    resetShellState();
  }

  function flushBufferedOutput() {
    if (session.lineBuffer) {
      send(ws, { type: "output", stream: "stdout", data: toCrlf(session.lineBuffer) });
      session.lineBuffer = "";
    }
  }

  function processShellData(chunk) {
    const raw = chunk.toString();
    session.lineBuffer += raw;

    if (!session.ready) {
      const readyIdx = session.lineBuffer.indexOf("__SAM_READY__");
      if (readyIdx >= 0) {
        const before = session.lineBuffer.slice(0, readyIdx).replace(/\n?__SAM_READY__\n?/g, "");
        if (before) send(ws, { type: "output", stream: "stdout", data: toCrlf(before) });
        session.ready = true;
        session.lineBuffer = session.lineBuffer.slice(readyIdx + "__SAM_READY__".length);
        processQueue();
      } else {
        send(ws, { type: "output", stream: "stdout", data: toCrlf(raw) });
      }
      return;
    }

    for (;;) {
      const { before, marker, after } = extractExitMarker(session.lineBuffer);
      session.lineBuffer = after;

      if (before) {
        send(ws, { type: "output", stream: "stdout", data: toCrlf(before) });
      }
      if (!marker) break;

      if (session.active && session.active.id === marker.id) {
        send(ws, {
          type: "exit",
          code: Number.isFinite(marker.code) ? marker.code : 0,
          cwd: marker.cwd,
        });
        session.active = null;
        processQueue();
      }
    }
  }

  function processQueue() {
    if (!session.shell || !session.ready || session.active || session.queue.length === 0) {
      return;
    }

    const next = session.queue.shift();
    session.active = next;
    send(ws, {
      type: "status",
      message: `Running on ${next.host}: ${next.command}`,
    });

    session.shell.write(`${next.command}\n`);
    session.shell.write(`printf "__SAM_EXIT__:${next.id}:%s:%s\\n" $? "$(pwd)"\n`);
  }

  function ensureShellConnection({ host, port, username, password }) {
    return new Promise((resolve, reject) => {
      const requestedKey = `${username}@${host}:${port}`;

      if (session.shell && session.connectionKey === requestedKey) {
        resolve();
        return;
      }

      if (session.shell || session.ssh) {
        closeShellSession();
      }

      const ssh = new Client();
      session.ssh = ssh;

      ssh
        .on("ready", () => {
          ssh.shell(
            { term: "xterm-256color", cols: 80, rows: 24 },
            (err, stream) => {
              if (err) {
                send(ws, {
                  type: "error",
                  message: `SSH shell failed: ${err.message}`,
                });
                closeShellSession();
                reject(err);
                return;
              }

              session.shell = stream;
              session.connectionKey = requestedKey;
              session.ready = false;

              stream.on("data", processShellData);

              stream.on("close", () => {
                flushBufferedOutput();
                send(ws, { type: "status", message: "Terminal shell closed." });
                closeShellSession();
              });

              stream.stderr.on("data", (data) => {
                send(ws, {
                  type: "output",
                  stream: "stderr",
                  data: toCrlf(data.toString()),
                });
              });

              send(ws, {
                type: "status",
                message: `Shell connected to ${requestedKey}`,
              });

              if (!SKIP_SCRIPT_WRAPPER) {
                stream.write(
                  "if command -v script >/dev/null 2>&1; then exec script -q -c bash /dev/null; fi\n"
                );
              }

              stream.write(
                "stty sane 2>/dev/null; stty isig icanon echo 2>/dev/null; stty intr $'\\x03' 2>/dev/null; true\n"
              );
              stream.write("export TERM=xterm-256color\n");
              stream.write("unset PROMPT_COMMAND\n");
              stream.write("export PS1='\\u@\\h:\\w$ '\n");
              stream.write("printf '__SAM_READY__\\n'\n");

              resolve();
            }
          );
        })
        .on("error", (err) => {
          send(ws, {
            type: "error",
            message: `SSH connection failed: ${err.message}`,
          });
          closeShellSession();
          reject(err);
        })
        .connect({
          host,
          port,
          username,
          password,
          readyTimeout: 15000,
        });
    });
  }

  function runSshExec({ host, port, username, password, command }) {
    return new Promise((resolve, reject) => {
      const ssh = new Client();
      let stdout = "";
      let stderr = "";

      ssh
        .on("ready", () => {
          ssh.exec(command, (err, stream) => {
            if (err) {
              ssh.end();
              reject(err);
              return;
            }

            stream.on("data", (data) => {
              stdout += data.toString();
            });

            stream.stderr.on("data", (data) => {
              stderr += data.toString();
            });

            stream.on("close", () => {
              ssh.end();
              resolve({ stdout, stderr });
            });
          });
        })
        .on("error", (err) => {
          reject(err);
        })
        .connect({
          host,
          port,
          username,
          password,
          readyTimeout: 15000,
        });
    });
  }

  function streamChildOutput(child, ws, progressType) {
    let stdoutBuffer = "";
    let stderrBuffer = "";

    function flushLines(kind) {
      const source = kind === "stderr" ? stderrBuffer : stdoutBuffer;
      const lines = source.split(/\r?\n/);
      const keep = lines.pop() || "";
      for (const line of lines) {
        const text = line.trim();
        if (!text) continue;
        send(ws, { type: progressType, stream: kind, message: text });
      }
      if (kind === "stderr") stderrBuffer = keep;
      else stdoutBuffer = keep;
    }

    child.stdout.on("data", (chunk) => {
      stdoutBuffer += chunk.toString("utf8");
      flushLines("stdout");
    });

    child.stderr.on("data", (chunk) => {
      stderrBuffer += chunk.toString("utf8");
      flushLines("stderr");
    });

    child.on("close", () => {
      if (stdoutBuffer.trim()) {
        send(ws, { type: progressType, stream: "stdout", message: stdoutBuffer.trim() });
      }
      if (stderrBuffer.trim()) {
        send(ws, { type: progressType, stream: "stderr", message: stderrBuffer.trim() });
      }
    });
  }

  function runPicoWebreplCommand({
    ws,
    action,
    host,
    webreplPort,
    webreplPassword,
    localPath,
    remotePath,
    resetAfter,
    onSuccess,
  }) {
    if (!fs.existsSync(PICO_WEBREPL_CLIENT_PATH)) {
      send(ws, {
        type: "pico_deploy_error",
        action,
        message: `Missing WebREPL helper script: ${PICO_WEBREPL_CLIENT_PATH}`,
      });
      return;
    }

    const args =
      action === "reset"
        ? [
            PICO_WEBREPL_CLIENT_PATH,
            "reset",
            "--host",
            host,
            "--port",
            String(webreplPort),
            "--password",
            webreplPassword,
          ]
        : action === "download"
        ? [
            PICO_WEBREPL_CLIENT_PATH,
            "get",
            "--host",
            host,
            "--port",
            String(webreplPort),
            "--password",
            webreplPassword,
            "--src",
            remotePath,
            "--dst",
            localPath,
          ]
        : [
            PICO_WEBREPL_CLIENT_PATH,
            "put",
            "--host",
            host,
            "--port",
            String(webreplPort),
            "--password",
            webreplPassword,
            "--src",
            localPath,
            "--dst",
            remotePath,
            ...(resetAfter ? ["--reset-after"] : []),
          ];

    send(ws, {
      type: "pico_deploy_started",
      action,
      host,
      webrepl_port: webreplPort,
      local_path: localPath || "",
      remote_path: remotePath || "",
    });

    const child = spawn("python3", args, {
      cwd: __dirname,
      stdio: ["ignore", "pipe", "pipe"],
    });

    streamChildOutput(child, ws, "pico_deploy_progress");

    child.on("error", (err) => {
      send(ws, {
        type: "pico_deploy_error",
        action,
        message: `Failed to start Pico deploy helper: ${err.message}`,
      });
    });

    child.on("close", (code) => {
      if (code === 0) {
        send(ws, {
          type: "pico_deploy_complete",
          action,
          host,
          webrepl_port: webreplPort,
          remote_path: remotePath || "",
        });
        if (typeof onSuccess === "function") {
          try {
            onSuccess();
          } catch (_err) {
            // ignore callback failures after success response
          }
        }
      } else {
        send(ws, {
          type: "pico_deploy_error",
          action,
          message: `${action} failed with exit code ${code}`,
        });
      }
    });
  }

  ws.on("message", (raw) => {
    let message;
    try {
      message = JSON.parse(raw.toString());
    } catch (_err) {
      send(ws, { type: "error", message: "Invalid JSON payload." });
      return;
    }

    const host = String(message.host || "").trim();
    const port = Number(message.port || 22);
    const username = String(message.username || "").trim();
    const password = String(message.password || "");
    const type = String(message.type || message.op || "").trim();

    if (type === "get_ui_layout" || type === "ui_layout_get") {
      send(ws, {
        type: "ui_layout_state",
        found: !!sharedUILayoutState,
        state: sharedUILayoutState,
      });
      return;
    }

    if (type === "set_ui_layout" || type === "ui_layout_set") {
      const state = message && typeof message === "object" ? message.state : null;
      if (!isValidUILayoutState(state)) {
        send(ws, {
          type: "ui_layout_error",
          message: "Invalid UI layout payload.",
        });
        return;
      }
      sharedUILayoutState = state;
      const ok = persistSharedUILayoutState(sharedUILayoutState);
      if (!ok) {
        send(ws, {
          type: "ui_layout_error",
          message: "Failed to persist shared UI layout.",
        });
        return;
      }
      send(ws, {
        type: "ui_layout_saved",
        savedAt: typeof state.savedAt === "string" ? state.savedAt : "",
      });
      return;
    }

    if (type === "get_walk_presets" || type === "walk_presets_get") {
      send(ws, {
        type: "walk_presets_state",
        presets: sharedWalkPresets,
      });
      return;
    }

    if (type === "set_walk_presets" || type === "walk_presets_set") {
      const presets = normalizeWalkPresets(message && typeof message === "object" ? message.presets : []);
      const ok = persistSharedWalkPresets(presets);
      if (!ok) {
        send(ws, {
          type: "walk_presets_error",
          message: "Failed to persist walk presets.",
        });
        return;
      }
      sharedWalkPresets = presets;
      send(ws, {
        type: "walk_presets_saved",
        count: sharedWalkPresets.length,
      });
      return;
    }

    if (type === "pico_file_read") {
      const rawLocalPath = String(message.local_path || "").trim();
      const localPath = rawLocalPath ? path.resolve(rawLocalPath) : "";
      if (!rawLocalPath) {
        send(ws, { type: "pico_file_error", action: "read", message: "local_path is required." });
        return;
      }
      try {
        const content = fs.readFileSync(localPath, "utf8");
        send(ws, {
          type: "pico_file_loaded",
          local_path: localPath,
          content,
        });
      } catch (err) {
        send(ws, {
          type: "pico_file_error",
          action: "read",
          message: `Failed to read file: ${err.message}`,
        });
      }
      return;
    }

    if (type === "pico_file_write") {
      const rawLocalPath = String(message.local_path || "").trim();
      const localPath = rawLocalPath ? path.resolve(rawLocalPath) : "";
      const content = typeof message.content === "string" ? message.content : "";
      if (!rawLocalPath) {
        send(ws, { type: "pico_file_error", action: "write", message: "local_path is required." });
        return;
      }
      try {
        fs.mkdirSync(path.dirname(localPath), { recursive: true });
        fs.writeFileSync(localPath, content, "utf8");
        send(ws, {
          type: "pico_file_saved",
          local_path: localPath,
          bytes: Buffer.byteLength(content, "utf8"),
        });
      } catch (err) {
        send(ws, {
          type: "pico_file_error",
          action: "write",
          message: `Failed to save file: ${err.message}`,
        });
      }
      return;
    }

    if (type === "pico_deploy" || type === "pico_upload" || type === "upload_pico_firmware") {
      const picoHost = String(message.pico_ip || message.ip || "").trim();
      const webreplPort = Number.parseInt(message.webrepl_port || 8266, 10);
      const webreplPassword = String(message.webrepl_password || "").trim();
      const rawLocalPath = String(message.local_path || "").trim();
      const localPath = rawLocalPath ? path.resolve(rawLocalPath) : "";
      const remotePath = String(message.remote_path || "/main.py").trim() || "/main.py";
      const resetAfter = !!message.reset_after;

      if (!picoHost || !webreplPassword || !rawLocalPath) {
        send(ws, {
          type: "pico_deploy_error",
          action: "upload",
          message: "pico_ip, webrepl_password, and local_path are required.",
        });
        return;
      }
      if (!Number.isFinite(webreplPort) || webreplPort < 1 || webreplPort > 65535) {
        send(ws, {
          type: "pico_deploy_error",
          action: "upload",
          message: "webrepl_port must be a valid TCP port.",
        });
        return;
      }
      if (!fs.existsSync(localPath) || !fs.statSync(localPath).isFile()) {
        send(ws, {
          type: "pico_deploy_error",
          action: "upload",
          message: `local_path is not a readable file: ${localPath}`,
        });
        return;
      }

      runPicoWebreplCommand({
        ws,
        action: "upload",
        host: picoHost,
        webreplPort,
        webreplPassword,
        localPath,
        remotePath,
        resetAfter,
      });
      return;
    }

    if (type === "pico_reset" || type === "reset_pico") {
      const picoHost = String(message.pico_ip || message.ip || "").trim();
      const webreplPort = Number.parseInt(message.webrepl_port || 8266, 10);
      const webreplPassword = String(message.webrepl_password || "").trim();

      if (!picoHost || !webreplPassword) {
        send(ws, {
          type: "pico_deploy_error",
          action: "reset",
          message: "pico_ip and webrepl_password are required.",
        });
        return;
      }
      if (!Number.isFinite(webreplPort) || webreplPort < 1 || webreplPort > 65535) {
        send(ws, {
          type: "pico_deploy_error",
          action: "reset",
          message: "webrepl_port must be a valid TCP port.",
        });
        return;
      }

      runPicoWebreplCommand({
        ws,
        action: "reset",
        host: picoHost,
        webreplPort,
        webreplPassword,
      });
      return;
    }

    if (type === "pico_file_pull" || type === "pico_pull_file") {
      const picoHost = String(message.pico_ip || message.ip || "").trim();
      const webreplPort = Number.parseInt(message.webrepl_port || 8266, 10);
      const webreplPassword = String(message.webrepl_password || "").trim();
      const remotePath = String(message.remote_path || "/main.py").trim() || "/main.py";

      if (!picoHost || !webreplPassword) {
        send(ws, {
          type: "pico_file_error",
          action: "pull",
          message: "pico_ip and webrepl_password are required.",
        });
        return;
      }
      if (!Number.isFinite(webreplPort) || webreplPort < 1 || webreplPort > 65535) {
        send(ws, {
          type: "pico_file_error",
          action: "pull",
          message: "webrepl_port must be a valid TCP port.",
        });
        return;
      }

      const tempPath = path.join(
        os.tmpdir(),
        `sam-pico-pull-${Date.now()}-${Math.random().toString(16).slice(2)}.py`
      );
      runPicoWebreplCommand({
        ws,
        action: "download",
        host: picoHost,
        webreplPort,
        webreplPassword,
        localPath: tempPath,
        remotePath,
        onSuccess: () => {
          try {
            const content = fs.readFileSync(tempPath, "utf8");
            send(ws, {
              type: "pico_file_pulled",
              remote_path: remotePath,
              content,
              pico_ip: picoHost,
            });
          } catch (err) {
            send(ws, {
              type: "pico_file_error",
              action: "pull",
              message: `Failed to read pulled Pico file: ${err.message}`,
            });
          } finally {
            try {
              fs.unlinkSync(tempPath);
            } catch (_err) {
              // ignore
            }
          }
        },
      });
      return;
    }

    if (type === "interrupt" || type === "send_interrupt" || type === "sigint") {
      if (session.shell && session.ready) {
        sendPtyInterrupt(session.shell);
      }
      return;
    }

    if (type === "input_b64" && typeof message.data === "string") {
      if (session.shell && session.ready) {
        try {
          const buf = Buffer.from(String(message.data).replace(/\s+/g, ""), "base64");
          if (buf.length === 1 && buf[0] === 3) {
            sendPtyInterrupt(session.shell);
          } else if (buf.length) {
            session.shell.write(buf);
          }
        } catch (_err) {
          // ignore
        }
      }
      return;
    }

    if (type === "resize") {
      const cols = Number.parseInt(message.cols, 10);
      const rows = Number.parseInt(message.rows, 10);
      if (
        session.shell &&
        session.ready &&
        Number.isFinite(cols) &&
        Number.isFinite(rows) &&
        cols > 0 &&
        rows > 0
      ) {
        try {
          session.shell.setWindow(rows, cols, 0, 0);
        } catch (_err) {
          // ignore
        }
      }
      return;
    }

    if (type === "input" && (typeof message.data === "string" || Buffer.isBuffer(message.data))) {
      if (session.shell && session.ready) {
        const raw =
          typeof message.data === "string"
            ? message.data
            : message.data.toString("utf8");
        if (raw.length === 1 && raw.charCodeAt(0) === 3) {
          sendPtyInterrupt(session.shell);
        } else {
          session.shell.write(raw);
        }
      }
      return;
    }

    if (type === "connect") {
      if (!host || !username) {
        send(ws, { type: "error", message: "host and username are required." });
        return;
      }
      ensureShellConnection({ host, port, username, password })
        .then(() => {
          send(ws, { type: "status", message: `Shell ready for ${host}` });
        })
        .catch((err) => {
          send(ws, { type: "error", message: `Unable to connect: ${err.message}` });
        });
      return;
    }

    if (type === "run_command" || type === "runCommand") {
      const command = String(message.command || "").trim();
      if (!host || !username || !command) {
        send(ws, {
          type: "error",
          message: "host, username, and command are required.",
        });
        return;
      }

      send(ws, {
        type: "status",
        message: `Queueing command for ${host}`,
      });

      ensureShellConnection({ host, port, username, password })
        .then(() => {
          const id = nextCommandId++;
          session.queue.push({ id, host, command });
          processQueue();
        })
        .catch((err) => {
          send(ws, {
            type: "error",
            message: `Unable to run command: ${err.message}`,
          });
        });
      return;
    }

    if (
      type === "scan_devices" ||
      type === "scanDevices" ||
      type === "device_scan" ||
      type === "device-scan" ||
      type === "scan_hotspot"
    ) {
      if (!host || !username) {
        send(ws, {
          type: "device_scan_error",
          message: "host and username are required.",
        });
        return;
      }

      // Build list of all IPs to check: known IPs + any new ones from ARP (minus blocked)
      const allIPs = [...knownHotspotIPs].filter((ip) => !blockedIPs.has(ip)).sort();
      const ipListArg = allIPs.join(" ");

      // Script: discover new 10.42.0.x IPs from ARP, merge with known list, ping all
      const scanScript = [
        "#!/bin/bash",
        "KNOWN=\"" + ipListArg + "\"",
        "HIF=$(ip -4 addr show | awk '/inet 10\\.42\\.0\\./{print $NF; exit}')",
        "",
        "# Discover new IPs from ARP table",
        "NEW=\"\"",
        '[ -n "$HIF" ] && NEW=$(ip -4 neigh show dev $HIF 2>/dev/null | awk \'{print $1}\' | sort -u)',
        "",
        "# Merge known + new, deduplicate",
        "ALL=$(echo $KNOWN $NEW | tr ' ' '\\n' | sort -u)",
        "",
        "# Collect hostnames from lease files",
        "HOSTNAMES=\"\"",
        "for f in /var/lib/NetworkManager/dnsmasq-*.leases /var/lib/misc/dnsmasq.leases /tmp/dnsmasq.leases; do",
        "  [ -f \"$f\" ] && HOSTNAMES=\"$HOSTNAMES $(cat \"$f\")\"",
        "done",
        "",
        "# Ping each IP and report status (3 attempts, any success = connected)",
        "for ip in $ALL; do",
        '  case "$ip" in 10.42.0.*) ;; *) continue ;; esac',
        '  case "$ip" in 10.42.0.1) continue ;; esac',
        '  st="disconnected"',
        '  for i in 1 2 3; do ping -c1 -W1 "$ip" >/dev/null 2>&1 && st="connected" && break; done',
        "  hn=$(echo \"$HOSTNAMES\" | awk -v ip=\"$ip\" '{for(i=1;i<=NF;i++){if($i==ip && $(i+1)!=\"*\"){print $(i+1); exit}}}')",
        '  [ -z "$hn" ] && hn="-"',
        '  printf "%s,%s,%s\\n" "$hn" "$st" "$ip"',
        "done",
      ].join("\n");
      const b64 = Buffer.from(scanScript).toString("base64");
      const script = `echo ${b64} | base64 -d | bash`;

      console.log("[scan] Known IPs:", allIPs.length ? allIPs.join(", ") : "(none yet)");

      const scanTimeout = new Promise((_, reject) =>
        setTimeout(() => reject(new Error("Scan timed out after 30s")), 30000)
      );

      Promise.race([
        runSshExec({ host, port, username, password, command: script }),
        scanTimeout,
      ])
        .then((result) => {
          const stdout = (result && result.stdout) || "";

          const devices = stdout
            .split(/\r?\n/)
            .map((line) => line.trim())
            .filter(Boolean)
            .map((line) => {
              const [name, status, ip] = line.split(",");
              return {
                name: name && name !== "-" ? name : "",
                status: status || "unknown",
                ip: ip || "-",
              };
            });

          // Filter out blocked IPs, update known set with newly discovered
          const filtered = devices.filter((d) => !blockedIPs.has(d.ip));
          for (const d of filtered) {
            if (d.ip && d.ip.startsWith("10.42.0.") && d.status === "connected") {
              knownHotspotIPs.add(d.ip);
            }
          }

          console.log("[scan] Results:", filtered.map((d) => `${d.ip}=${d.status}`).join(", ") || "(empty)");
          send(ws, { type: "device_scan_result", devices: filtered });
        })
        .catch((err) => {
          console.error("[scan] Error:", err.message);
          send(ws, {
            type: "device_scan_error",
            message: `Device scan failed: ${err.message}`,
          });
        });
      return;
    }

    // Ignore unknown/heartbeat message types to avoid UI spam.
  });

  ws.on("close", () => {
    closeShellSession();
  });
});

console.log(`Terminal gateway listening on ws://localhost:${PORT}`);
