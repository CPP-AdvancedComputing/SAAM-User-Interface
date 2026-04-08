import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const DEG = Math.PI / 180;
const LEG_LENGTH = 1.2;
const LEG_WIDTH = 0.07;
const LEG_DEPTH = 0.06;
/** Joint lies 75% up from the foot; offset mesh so that point sits on the parent pivot. */
const JOINT_FRAC = 0.75;
const MESH_CENTER_Y = -((JOINT_FRAC - 0.5) * LEG_LENGTH);

const INNER_SCALE = 0.018;
const OUTER_SCALE = 0.018;

const LEG_COLORS = [0xe11d48, 0x22c55e, 0x3b82f6, 0xca8a04];
const PARALLEL_X = [-0.2, -0.067, 0.067, 0.2];

/**
 * @typedef {Object} LegPose
 * @property {string} id
 * @property {number} inner_stepper
 * @property {number} outer_stepper
 * @property {number} hip
 * @property {number} yaw
 * @property {number} pitch
 * @property {number} roll
 */

/**
 * @param {object} options
 * @param {HTMLElement | null} options.panel Draggable `quick-panel` (not a modal dialog).
 * @param {() => Record<string, LegPose>} options.getPose
 */
export function initRobotRenderUI({ panel, getPose }) {
  if (!panel || typeof getPose !== "function") return;

  const wrap = panel.querySelector("#robot-render-canvas-wrap");
  const closeBtn = panel.querySelector("#robot-render-close-btn");
  const fullscreenBtn = panel.querySelector("#robot-render-fullscreen-btn");

  if (!wrap) return;

  let renderer = null;
  let scene = null;
  let camera = null;
  let controls = null;
  /** @type {ReturnType<typeof makeLeg>[] | null} */
  let legEntries = null;
  let rafId = 0;
  /** @type {ResizeObserver | null} */
  let resizeObs = null;

  /** @param {number} color */
  function makeLeg(color) {
    const legRoot = new THREE.Group();
    const baseYaw = new THREE.Group();
    const hipG = new THREE.Group();
    const yawG = new THREE.Group();
    const pitchG = new THREE.Group();
    const rollG = new THREE.Group();
    const extG = new THREE.Group();
    const meshG = new THREE.Group();

    const geo = new THREE.BoxGeometry(LEG_WIDTH, LEG_LENGTH, LEG_DEPTH);
    const mat = new THREE.MeshStandardMaterial({
      color,
      metalness: 0.18,
      roughness: 0.62,
    });
    const mesh = new THREE.Mesh(geo, mat);
    meshG.add(mesh);
    meshG.position.y = MESH_CENTER_Y;

    extG.add(meshG);
    rollG.add(extG);
    pitchG.add(rollG);
    yawG.add(pitchG);
    hipG.add(yawG);
    baseYaw.add(hipG);
    legRoot.add(baseYaw);

    return { legRoot, baseYaw, hipG, yawG, pitchG, rollG, extG };
  }

  function applyLayout() {
    if (!legEntries) return;
    const mode = "parallel";
    legEntries.forEach((e, i) => {
      if (mode === "spread") {
        e.legRoot.position.set(0, 0, 0);
        e.baseYaw.rotation.set(0, i * (Math.PI / 2), 0);
      } else {
        e.legRoot.position.set(PARALLEL_X[i] ?? 0, 0, 0);
        e.baseYaw.rotation.set(0, 0, 0);
      }
    });
  }

  function onResize() {
    if (!renderer || !camera || !wrap || panel.hidden) return;
    const w = Math.max(220, Math.floor(wrap.clientWidth));
    const h = Math.max(280, Math.floor(wrap.clientHeight));
    if (w <= 0 || h <= 0) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }

  function buildScene() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0f18);

    camera = new THREE.PerspectiveCamera(48, 1, 0.08, 80);
    camera.position.set(2.1, 1.35, 2.5);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    wrap.innerHTML = "";
    wrap.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, -0.45, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;

    scene.add(new THREE.AmbientLight(0xffffff, 0.38));
    const key = new THREE.DirectionalLight(0xffffff, 0.88);
    key.position.set(3.2, 5.5, 2.8);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xb6c4ff, 0.25);
    fill.position.set(-2.5, 1.2, -2);
    scene.add(fill);

    const hubGeo = new THREE.SphereGeometry(0.048, 20, 16);
    const hubMat = new THREE.MeshStandardMaterial({
      color: 0x64748b,
      metalness: 0.35,
      roughness: 0.45,
    });
    const hub = new THREE.Mesh(hubGeo, hubMat);
    scene.add(hub);

    const grid = new THREE.GridHelper(4, 20, 0x334155, 0x1e293b);
    grid.position.y = -0.85;
    scene.add(grid);

    legEntries = LEG_COLORS.map((c) => makeLeg(c));
    legEntries.forEach((e) => scene.add(e.legRoot));
    applyLayout();

    resizeObs = new ResizeObserver(() => onResize());
    resizeObs.observe(wrap);
    window.addEventListener("resize", onResize);
    onResize();
  }

  function disposeScene() {
    window.removeEventListener("resize", onResize);
    resizeObs?.disconnect();
    resizeObs = null;
    if (controls) {
      controls.dispose();
      controls = null;
    }
    if (renderer) {
      renderer.dispose();
      if (renderer.domElement.parentNode === wrap) wrap.removeChild(renderer.domElement);
      renderer = null;
    }
    scene = null;
    camera = null;
    legEntries = null;
  }

  function tick() {
    rafId = requestAnimationFrame(tick);
    if (!legEntries || !renderer || !scene || !camera) return;

    const poseMap = getPose();
    legEntries.forEach((e, i) => {
      const key = `l${i}`;
      const p = poseMap[key];
      if (!p) return;

      e.hipG.rotation.set(0, p.hip * DEG, 0);
      e.yawG.rotation.set(0, p.yaw * DEG, 0);
      e.pitchG.rotation.set(p.pitch * DEG, 0, 0);
      e.rollG.rotation.set(0, 0, p.roll * DEG);

      e.extG.position.set(
        p.outer_stepper * OUTER_SCALE,
        -p.inner_stepper * INNER_SCALE,
        0
      );
    });

    controls?.update();
    renderer.render(scene, camera);
  }

  function startLoop() {
    if (rafId) return;
    if (!scene) buildScene();
    else onResize();
    rafId = requestAnimationFrame(tick);
  }

  function stopLoop() {
    if (rafId) cancelAnimationFrame(rafId);
    rafId = 0;
  }

  function syncRendererToPanelVisibility() {
    if (panel.hidden) {
      if (document.fullscreenElement === panel) {
        document.exitFullscreen?.();
      }
      stopLoop();
      disposeScene();
    } else {
      startLoop();
    }
  }

  function syncFullscreenButtonState() {
    if (!fullscreenBtn) return;
    const isFs = document.fullscreenElement === panel;
    fullscreenBtn.textContent = isFs ? "Exit fullscreen" : "Fullscreen";
    fullscreenBtn.setAttribute("aria-pressed", String(isFs));
  }

  async function toggleFullscreen() {
    try {
      if (document.fullscreenElement === panel) {
        await document.exitFullscreen?.();
      } else {
        await panel.requestFullscreen?.();
      }
    } catch (_err) {
      // ignore
    }
  }

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

  const visibilityObserver = new MutationObserver(() => syncRendererToPanelVisibility());
  visibilityObserver.observe(panel, { attributes: true, attributeFilter: ["hidden"] });

  fullscreenBtn?.addEventListener("click", toggleFullscreen);
  document.addEventListener("keydown", (e) => {
    if (panel.hidden) return;
    if (isTypingContext(e.target)) return;
    // Space / Spacebar both covered for browser compatibility.
    if (e.key !== " " && e.key !== "Spacebar") return;
    e.preventDefault();
    toggleFullscreen();
  });
  document.addEventListener("fullscreenchange", () => {
    syncFullscreenButtonState();
    onResize();
  });
  syncFullscreenButtonState();

  closeBtn?.addEventListener("click", () => {
    panel.hidden = true;
    const launcherBtn = document.querySelector('[data-panel-target="robot-render-panel"]');
    if (launcherBtn) launcherBtn.setAttribute("aria-expanded", "false");
  });
}
