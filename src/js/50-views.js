// ---------------------------------------------------------------- flange path and working-range outlines in the robot plane (y = 0)
let flangePts = S.tcp.map((p) => [p[0], 0, p[1] + TOOL.len]);
function lineOf(points, opacity = 1) {
  const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(points.flat()), 3));
  const o = new THREE.Line(g, new THREE.LineBasicMaterial({ transparent: opacity < 1, opacity })); world.add(o); return o;
}
const pathGhost = lineOf(flangePts, 0.22), pathLive = lineOf(flangePts);
function rebuildPaths() {        // after a what-if rebuild: new flange path
  flangePts = S.tcp.map((p) => [p[0], 0, p[1] + TOOL.len]);
  for (const line of [pathGhost, pathLive]) {
    line.geometry.dispose();
    const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(flangePts.flat()), 3));
    line.geometry = g;
  }
}
const flangeTop = ROBOT.mountZ - ROBOT.datasheet.below;
const reqRect = lineOf([[-0.3, 0, flangeTop], [0.3, 0, flangeTop], [0.3, 0, flangeTop - 0.4], [-0.3, 0, flangeTop - 0.4], [-0.3, 0, flangeTop]]);
const dsOutline = (() => {
  const W = ROBOT.datasheet.width / 2, H = ROBOT.datasheet.height, D = ROBOT.datasheet.arcDepth;
  const R = (W * W + (D - H) * (D - H)) / (2 * (D - H)), zc = flangeTop - D + R;     // R = 1191.4 mm, as on the drawing
  const pts = [[-W, 0, flangeTop], [W, 0, flangeTop], [W, 0, flangeTop - H]];
  const a0 = Math.atan2(flangeTop - H - zc, W), a1 = Math.atan2(flangeTop - H - zc, -W);
  for (let i = 0; i <= 40; i++) { const a = lerp(a0, a1, i / 40); pts.push([R * Math.cos(a), 0, zc + R * Math.sin(a)]); }
  pts.push([-W, 0, flangeTop]);
  return { line: lineOf(pts, 0.5), R, pts };
})();

function applyTheme() {
  const top = new THREE.Color(css("--scene-top")), bottom = new THREE.Color(css("--scene-bottom"));
  const c = document.createElement("canvas"); c.width = 2; c.height = 256;
  const g = c.getContext("2d"), gr = g.createLinearGradient(0, 0, 0, 256);
  gr.addColorStop(0, "#" + top.getHexString()); gr.addColorStop(1, "#" + bottom.getHexString());
  g.fillStyle = gr; g.fillRect(0, 0, 2, 256);
  const tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace; scene.background = tex;
  scene.environmentIntensity = themeIsDark() ? 0.5 : 0.7;
  const robotC = new THREE.Color(css("--delta"));
  pathGhost.material.color.copy(robotC); pathLive.material.color.copy(robotC); reqRect.material.color.copy(robotC);
  dsOutline.line.material.color.set(css("--faint"));
  toneFloor();
  refreshCharts();
}

// ---------------------------------------------------------------- camera views (three.js frame: world (x, y, z) -> (x, z, -y))
const views = {
  iso: { pos: V(2.1, 1.3, -3.45), target: V(0, 0.42, 0) },
  front: { pos: V(0, 0.62, -3.7), target: V(0, 0.55, 0) },
  side: { pos: V(3.6, 0.7, 0.0), target: V(0, 0.45, 0) },
  top: { pos: V(0.0, 4.6, -0.02), target: V(0, 0, 0) },
  robot: { pos: V(-0.62, 0.72, -0.52), target: V(0.05, 1.02, 0) },
  pick: { pos: V(0.62, 0.55, -0.62), target: V(0.12, 0.16, 0) },
};
window.pcViews = views;
let flight = null;
function setView(name) {
  const v = views[name]; if (!v) return;
  flight = { t: 0, fromP: camera.position.clone(), fromT: controls.target.clone(), toP: v.pos.clone(), toT: v.target.clone() };
  document.querySelectorAll(".views button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.view === name)));
}
camera.position.copy(views.iso.pos); controls.target.copy(views.iso.target);
document.querySelectorAll(".views button").forEach((b) => b.addEventListener("click", () => setView(b.dataset.view)));
controls.addEventListener("start", () => { flight = null; document.querySelectorAll(".views button").forEach((b) => b.setAttribute("aria-pressed", "false")); });
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false); camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix(); drawLanes();
}
new ResizeObserver(resize).observe(viewport);
