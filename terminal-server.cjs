const { WebSocketServer } = require("ws");
const { Client } = require("ssh2");

const PORT = Number(process.env.TERMINAL_GATEWAY_PORT || 8787);
let nextCommandId = 1;

// Persistent set of known hotspot IPs (survives across scans)
const knownHotspotIPs = new Set();

// IPs to never show in device list
const blockedIPs = new Set(["10.42.0.106"]);

function stripAnsi(input) {
  return input
    .replace(/\u001b\][^\u0007]*\u0007/g, "")
    .replace(/\u001b\[[0-9;?]*[ -/]*[@-~]/g, "")
    .replace(/\r/g, "");
}

function send(ws, payload) {
  if (ws.readyState === 1) {
    ws.send(JSON.stringify(payload));
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
      send(ws, { type: "output", stream: "stdout", data: session.lineBuffer });
      session.lineBuffer = "";
    }
  }

  function processShellData(chunk) {
    session.lineBuffer += stripAnsi(chunk.toString());

    let newlineIndex = session.lineBuffer.indexOf("\n");
    while (newlineIndex !== -1) {
      const line = session.lineBuffer.slice(0, newlineIndex + 1);
      session.lineBuffer = session.lineBuffer.slice(newlineIndex + 1);

      if (!session.ready) {
        if (line.includes("__SAM_READY__")) {
          session.ready = true;
          processQueue();
        }
        newlineIndex = session.lineBuffer.indexOf("\n");
        continue;
      }

      const marker = line.match(/__SAM_EXIT__:(\d+):(-?\d+):(.*)\n?$/);
      if (marker) {
        const prefix = line.slice(0, marker.index || 0);
        if (prefix) {
          const cleanedPrefix = prefix.replace(/printf "__SAM_EXIT__:[^\n]*/g, "");
          if (cleanedPrefix.trim().length > 0) {
            send(ws, { type: "output", stream: "stdout", data: cleanedPrefix });
          }
        }

        const markerId = Number(marker[1]);
        const code = Number(marker[2]);
        const cwd = (marker[3] || "").trim();
        if (session.active && session.active.id === markerId) {
          send(ws, {
            type: "exit",
            code: Number.isFinite(code) ? code : 0,
            cwd,
          });
          session.active = null;
          processQueue();
        }
      } else if (line.includes(`printf "__SAM_EXIT__:`)) {
        // Suppress internal completion marker echo.
      } else {
        send(ws, { type: "output", stream: "stdout", data: line });
      }

      newlineIndex = session.lineBuffer.indexOf("\n");
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
          ssh.shell({ term: "xterm-color" }, (err, stream) => {
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
                data: data.toString(),
              });
            });

            send(ws, {
              type: "status",
              message: `Shell connected to ${requestedKey}`,
            });

            // Normalize shell behavior for cleaner web-terminal output.
            stream.write("stty -echo >/dev/null 2>&1 || true\n");
            stream.write("export TERM=dumb\n");
            stream.write("unset PROMPT_COMMAND\n");
            stream.write("export PS1=''\n");
            stream.write("set +o vi >/dev/null 2>&1 || true\n");
            stream.write(
              "bind 'set enable-bracketed-paste off' >/dev/null 2>&1 || true\n"
            );
            stream.write("alias ls='ls --color=never' >/dev/null 2>&1 || true\n");
            stream.write("printf '__SAM_READY__\\n'\n");

            resolve();
          });
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

    if (type === "interrupt" || type === "send_interrupt") {
      if (session.shell && session.ready) {
        session.shell.write("\x03");
      }
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

