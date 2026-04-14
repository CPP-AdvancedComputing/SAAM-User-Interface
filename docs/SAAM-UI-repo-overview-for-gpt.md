# S.A.A.M. Control Interface — Repository Overview

**Repo:** `sam-user-interface`  
**Purpose:** Browser-based operator UI for the S.A.A.M. quadruped stack (ROS commands, telemetry, SSH console to Jetson, optional 3D leg view).

---

## Tech stack

| Area | Technology |
|------|------------|
| Build | Vite 7 (`npm run dev`, `npm run build`, `npm run preview`) |
| App | Vanilla ES modules — primary `app.js` (~4k lines) + `robot-render.js` |
| ROS | `roslib` over WebSocket → rosbridge |
| 3D | three.js + OrbitControls (`robot-render.js`) |
| Terminal UI | xterm.js + `@xterm/addon-fit` |
| Terminal backend | Node: `ws` + `ssh2` (`terminal-server.cjs`) |
| Styles | `styles.css` (no React/Vue) |

**Scripts (package.json):**

- `dev` — Vite dev server  
- `build` / `preview` — production bundle  
- `terminal` — run `node terminal-server.cjs`  
- `dev:all` — gateway + Vite via `concurrently`

---

## Repository layout (high level)

- `index.html` — Single-page shell; optional URL modes `?autostart=render`, `?popout=commands`  
- `app.js` — ROS connection, commands, logs, listen panel, trajectory/planner, IMU/gyro, terminal tabs, layout persistence, etc.  
- `robot-render.js` — Three.js leg visualization  
- `styles.css` — Global styling  
- `terminal-server.cjs` — WebSocket gateway: SSH shell, resize, interrupt, hotspot device scan, optional rosbridge launch helper  
- `dist/` — Vite build output (for static hosting)

There is **no root README** in the repo as of this document; behavior is defined by code and HTML.

---

## Configuration

### In code (`app.js` — `CONFIG`)

- **ROS:** `bridgeUrl`, topics such as `/sam/command`, `/web/command`, `/planner/*`, `/web/log`, per-leg IMU paths, Jetson CPU topics, message types (`std_msgs`, `sam_interfaces`, etc.)  
- **Legs:** `l0`–`l3`; joints: inner/outer stepper + servo  
- **Terminal gateway:** default WebSocket URL (often same host as robot, port **8787**)

### Environment variables

- **`VITE_TERMINAL_PASSWORD`** — Optional default SSH password baked at build time  
- **`TERMINAL_GATEWAY_PORT`** — Gateway listen port (default 8787)  
- **`SAM_SKIP_SCRIPT_WRAPPER`** — Set to `1` to disable wrapping the SSH shell in `script(1)` (used so `sudo` can open `/dev/tty` on some PTY stacks)

### Browser storage

- ROS bridge URL and terminal gateway URL persisted (keys like `sam-ui-ros-bridge-url-v1`, `sam-ui-terminal-gateway-url-v1`)  
- UI layout / panel positions may be saved for restore

---

## Major features

### Connection & safety

- Header shows **Terminal** and **ROS** status  
- **Launch** — Sends a command via gateway to start rosbridge on the Jetson (when connected)  
- **E-Stop / Start** — Publish emergency-stop style payloads (`std_msgs` String)  
- Backtick key bound to E-Stop (global)

### Commands panel

- Form builder for command types (move, servo, enable, query, home, rate, raw, walk, …)  
- Per-leg targeting **l0–l3**  
- Send to configured ROS topics; save presets  
- **Pop out** — Full-window commands mode via `?popout=commands`

### Hotspot devices

- List built from **SSH scan** on Jetson (`scan_hotspot` / similar messages to gateway)  
- **Legs / All** (or similar) tab views  
- Display names can be mapped by IP (e.g. leg labels); some IPs can be blocked (UI + server)

### Event log

- Filters: All, Errors, ROS, Sent, **Pico**  
- **Pico** view: subscribes to **`/web/log`** (`std_msgs/String`); payload can be **NDJSON** (one JSON object per line) with fields like `source_ip`, `leg_id`, `message` — buffered per leg **l0–l3**

### Listen panel

- Preset ROS topic listeners (web log, planner status, Jetson metrics, per-leg IMU, enabled state, …)  
- Custom topic + type  
- Optional **UDP** listener mode for raw packets

### Trajectory & planner

- Topics: global pose sequence, leg command, planner status  
- UI for stepping / status display (see `app.js` for current wiring)

### Per-leg IMU / gyro

- Subscriptions under `/l0`–`/l3` (filtered IMU, roll/pitch strings, raw, zero commands)  
- Display and zeroing for stance/readouts; ties into render where applicable

### Leg enable

- Topics like `/lN/enabled_state` and commands to toggle legs

### Jetson telemetry

- CPU temperature and load (Float32 topics)

### 3D render panel

- Leg poses from live snapshot; draggable panel; fullscreen  
- **`?autostart=render`** — Full-viewport render-focused mode

### Console (terminal)

- **Multi-tab** xterm sessions  
- WebSocket to **terminal-server** → **SSH** to Jetson (host/user/password from UI)  
- PTY resize forwarded to SSH; debounced/throttled resize messages; deduplicated PTY size updates  
- Ctrl+C sends interrupt to remote shell  
- Gateway may use **`script -c bash`** wrapper unless disabled (sudo password prompts)

---

## `terminal-server.cjs` responsibilities

- Accept WebSocket clients; JSON message types include: `connect`, `input`, `input_b64`, `resize`, `interrupt`, `run_command`, `scan_hotspot` / device scan, rosbridge-related launch, etc.  
- Maintain SSH session + shell stream; forward stdout/stderr to browser as JSON `output`  
- Run remote bash over SSH for device discovery (ARP/lease parsing, ping, CSV-style results)  
- Optional: **`SAM_SKIP_SCRIPT_WRAPPER`** to skip `script` wrapper

---

## Deployment notes

- **Development:** `npm install`, then `npm run dev` (and run `terminal` separately or `npm run dev:all`)  
- **Production UI:** `npm run build` → serve `dist/` behind any static host; ensure **rosbridge** and **terminal gateway** URLs are reachable from the browser (CORS/WebSocket as needed)  
- **Jetson:** SSH access for terminal; rosbridge on configured port (e.g. 9090); firewall rules for bridge + gateway ports

---

## Known gaps

- No first-class README for onboarding  
- `app.js` is large and monolithic  
- Default IPs in `CONFIG` / HTML are site-specific; operators should set bridge/gateway via UI or env

---

*Generated as project context for GPT / documentation. Adjust dates and details if the repo diverges.*
