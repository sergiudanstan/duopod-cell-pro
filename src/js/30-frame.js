// Floor and markings, portal frame from real 80x80 slot-8 profiles, mounting plate, acrylic guards with a door,
// light curtains at the belt openings, electrical cabinet, air preparation unit, cable tray, stack light, HMI.
const floorMat = mat(0x8a8e92, 0.92, 0.02);
if (M.floor.map) { const fm = M.floor.map.clone(); fm.repeat.set(24, 24); floorMat.map = fm; }
const floor = mesh(new THREE.PlaneGeometry(14, 14), floorMat);
floor.position.z = LAYOUT.floor; floor.castShadow = false;
function toneFloor() { floorMat.color.set(themeIsDark() ? 0x3a3e42 : 0x8a8e92); }
window.addEventListener("themechange", () => queueMicrotask(toneFloor));
queueMicrotask(toneFloor);
function tapeStrip(x, y, w, d, m = M.tape) { const o = mesh(new THREE.PlaneGeometry(w, d), m); o.position.set(x, y, LAYOUT.floor + 0.003); o.castShadow = false; return o; }
const PX = LAYOUT.posts.x, PY = LAYOUT.posts.y, [TZ0, TZ1] = LAYOUT.topZ, TZ = (TZ0 + TZ1) / 2;
{
  // hazard tape around the cell, walkway lines in front of the door and the cabinet
  const lx = PX + 0.20, ly = PY + 0.20, w = 0.05;
  tapeStrip(lx, 0, w, ly * 2 + w); tapeStrip(-lx, 0, w, ly * 2 + w);
  for (const sy of [-1, 1]) for (const sx of [-1, 1]) tapeStrip(sx * (0.42 + lx) / 2, sy * ly, lx - 0.42, w);   // ends, leaving the belt openings free
  const walk = mat(0x2f8a4a, 0.8, 0); walk.color.convertSRGBToLinear();
  tapeStrip(-PX - 0.95, 0, 0.04, 2.6, walk); tapeStrip(-PX - 0.25, 0.0, 0.04, 2.6, walk);
}

// ---------------------------------------------------------------- 80x80 slot-8 profile section (two slots per face), metres
function profile8080Shape() {
  const s = 0.001, r = 4, sh = new THREE.Shape();
  const face = [[20, -4], [15.5, -4], [15.5, -8], [9.75, -8], [7.75, -6], [7.75, 6], [9.75, 8], [15.5, 8], [15.5, 4], [20, 4]];
  for (let k = 0; k < 4; k++) {
    const a = k * Math.PI / 2, c = Math.cos(a), si = Math.sin(a);
    const R = ([x, y]) => [(x * c - y * si) * s, (x * si + y * c) * s];
    const [cx, cy] = R([36, -36]);
    if (k === 0) sh.moveTo(...R([36, -40]));
    sh.absarc(cx, cy, r * s, a - Math.PI / 2, a, false);
    for (const off of [-20, 20]) for (const [x, y] of face) sh.lineTo(...R([x + 20, y + off]));
    sh.lineTo(...R([40, 36]));
  }
  sh.closePath();
  const hole = (x, y, rad) => { const p = new THREE.Path(); p.absarc(x * s, y * s, rad * s, 0, Math.PI * 2, true); return p; };
  sh.holes.push(hole(0, 0, 7.0));
  for (const [x, y] of [[20, 20], [-20, 20], [-20, -20], [20, -20]]) sh.holes.push(hole(x, y, 3.4));
  for (const [x, y] of [[0, 26], [0, -26], [26, 0], [-26, 0]]) sh.holes.push(hole(x, y, 2.6));
  return sh;
}
const profile8080 = makeInstanced(new THREE.ExtrudeGeometry(profile8080Shape(), { depth: 1, bevelEnabled: false, curveSegments: 4 }), M.aluClear, 96, world);
const endCaps = makeInstanced(new RoundedBoxGeometry(0.08, 0.08, 0.006, 2, 0.003), M.blackPlastic, 96, world);
function beam80(a, b, caps = true) {
  const d = b.clone().sub(a), L = d.length(), u = d.clone().normalize();
  _beamQ.setFromUnitVectors(_zAxis, u); _beamS.set(1, 1, L); _beamM.compose(a, _beamQ, _beamS);
  profile8080.setMatrixAt(profile8080.count++, _beamM);
  if (caps) for (const [p, sg] of [[a, -1], [b, 1]]) { _beamS.set(1, 1, 1); _beamM.compose(p.clone().addScaledVector(u, sg * 0.003), _beamQ, _beamS); endCaps.setMatrixAt(endCaps.count++, _beamM); }
}
// angle bracket 80 x 80 (die-cast look) with two bolts per leg
const bracketGeo = (() => { const g = BufferGeometryUtils.mergeGeometries([new THREE.BoxGeometry(0.075, 0.008, 0.075).translate(0, 0, 0), new THREE.BoxGeometry(0.075, 0.075, 0.008).translate(0, 0.0335, -0.0335)]); return g; })();
const brackets = makeInstanced(bracketGeo, M.zinc9006, 64, world);
function bracketAt(pos, quat) { _beamS.set(1, 1, 1); _beamM.compose(pos, quat, _beamS); brackets.setMatrixAt(brackets.count++, _beamM); }

for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
  beam80(V(sx * PX, sy * PY, LAYOUT.floor + 0.012), V(sx * PX, sy * PY, TZ1));
  rbox(0.16, 0.16, 0.012, 0.002, M.galv, sx * PX, sy * PY, LAYOUT.floor + 0.006, world);
  for (const [dx, dy] of [[0.055, 0.055], [-0.055, 0.055], [0.055, -0.055], [-0.055, -0.055]]) addBolt(V(sx * PX + dx, sy * PY + dy, LAYOUT.floor + 0.012), V(0, 0, 1));
  // levelling foot under the plate
  lathe([[0, LAYOUT.floor - 0.0], [0.045, LAYOUT.floor], [0.045, LAYOUT.floor + 0.002], [0, LAYOUT.floor + 0.002]], M.blackRubber, world).position.set(sx * PX, sy * PY, 0);
}
for (const sy of [-1, 1]) beam80(V(-PX - 0.04, sy * PY, TZ), V(PX + 0.04, sy * PY, TZ));
for (const sx of [-1, 1]) beam80(V(sx * PX, -PY + 0.04, TZ), V(sx * PX, PY - 0.04, TZ), false);
for (const sx of [-1, 1]) beam80(V(sx * LAYOUT.carriers, -PY + 0.04, TZ), V(sx * LAYOUT.carriers, PY - 0.04, TZ), false);
const gz0 = LAYOUT.floor + 0.14, gz1 = LAYOUT.guardTop - 0.10;
for (const sx of [-1, 1]) beam80(V(sx * PX, -PY + 0.04, gz0 - 0.04), V(sx * PX, PY - 0.04, gz0 - 0.04), false);
// brackets under the top beams at the posts and the carriers
{
  const q = new THREE.Quaternion();
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    q.setFromEuler(new THREE.Euler(0, 0, sy > 0 ? Math.PI : 0)); bracketAt(V(sx * PX, sy * (PY - 0.04), TZ0 - 0.004), q);
    q.setFromEuler(new THREE.Euler(0, 0, sy > 0 ? Math.PI : 0)); bracketAt(V(sx * LAYOUT.carriers, sy * (PY - 0.04), TZ0 - 0.004), q);
  }
}
// mounting plate: underside is the robot mounting face (z = ROBOT.mountZ); M20 screws per the connecting dimensions
rbox(LAYOUT.plate.x, LAYOUT.plate.y, LAYOUT.plate.t, 0.004, M.aluBlasted, 0, 0, ROBOT.mountZ + LAYOUT.plate.t / 2, world);
for (const sx of [-1, 1]) for (const y of [-0.16, 0, 0.16]) addBolt(V(sx * LAYOUT.carriers, y, TZ1), V(0, 0, 1));
for (const [x, y] of [[-0.2, 0], [0.2, 0], [-0.17, -0.105], [0.17, -0.105], [-0.17, 0.105], [0.17, 0.105]]) {
  const h = lathe([[0, 0], [0.016, 0], [0.016, 0.012], [0, 0.012]], M.steel, world, 6); h.position.set(x, y, ROBOT.mountZ + LAYOUT.plate.t);
}

// ---------------------------------------------------------------- acrylic (plexiglass) guards in black frames, door with hinges and safety switch
// clear PMMA (acrylic) 6 mm in black frames; slight blue edge tint and a faint reflection, like real guarding
const plexiMat = phys(0xf4f8fb, 0.04, 0, {
  transparent: true, opacity: 0.10, depthWrite: false, side: THREE.DoubleSide,
  clearcoat: 1, clearcoatRoughness: 0.05, envMapIntensity: 1.4,
});
const guardGroup = new THREE.Group(); world.add(guardGroup);
function meshPanel(w, h, x, y, z, rotZ, parent = guardGroup) {
  const g = new THREE.Group(); g.position.set(x, y, z); g.rotation.z = rotZ; parent.add(g);
  const pane = mesh(new THREE.BoxGeometry(w - 0.02, 0.006, h - 0.02), plexiMat, g); pane.castShadow = false; pane.renderOrder = 2;
  const f = 0.02;
  rbox(w, f, f, 0.002, M.powderBlack, 0, 0, h / 2 - f / 2, g); rbox(w, f, f, 0.002, M.powderBlack, 0, 0, -h / 2 + f / 2, g);
  rbox(f, f, h, 0.002, M.powderBlack, -w / 2 + f / 2, 0, 0, g); rbox(f, f, h, 0.002, M.powderBlack, w / 2 - f / 2, 0, 0, g);
  return g;
}
const gh = gz1 - gz0, gzc = (gz0 + gz1) / 2;
// +x side: two fixed panels; -x side: fixed panel + hinged door (operator side of the HMI)
meshPanel(PY - 0.06, gh, PX + 0.05, -PY / 2, gzc, Math.PI / 2); meshPanel(PY - 0.06, gh, PX + 0.05, PY / 2, gzc, Math.PI / 2);
meshPanel(PY - 0.06, gh, -PX - 0.05, -PY / 2, gzc, Math.PI / 2);
const door = meshPanel(PY - 0.08, gh - 0.02, -PX - 0.055, PY / 2, gzc, Math.PI / 2);
{
  for (const dz of [-0.55, 0.0, 0.55]) { const hg = vcyl(0.010, 0.07, M.powderBlack, -PX - 0.055, 0.06, gzc + dz, 16, guardGroup); void hg; }   // hinges
  const handle = rod(0.008, M.chrome, 12); guardGroup.add(handle); placeRod(handle, V(-PX - 0.085, PY - 0.08, gzc - 0.12), V(-PX - 0.085, PY - 0.08, gzc + 0.12));
  for (const dz of [-0.12, 0.12]) { const st = rod(0.006, M.chrome, 10); guardGroup.add(st); placeRod(st, V(-PX - 0.06, PY - 0.08, gzc + dz), V(-PX - 0.085, PY - 0.08, gzc + dz)); }
  rbox(0.035, 0.03, 0.09, 0.004, M.estopYellow, -PX - 0.07, PY - 0.035, gzc + 0.30, guardGroup);      // safety switch body
  rbox(0.02, 0.025, 0.035, 0.003, M.blackPlastic, -PX - 0.07, PY - 0.055, gzc + 0.30, guardGroup);     // actuator
  const sign = labelPlane("⚠ AUTO MODE", 0.16, 0.05, { bg: "#f2c400", fg: "#111", font: "700 96px 'IBM Plex Sans', sans-serif" });
  sign.rotation.set(Math.PI / 2, -Math.PI / 2, 0); sign.position.set(-PX - 0.068, PY / 2, gzc + 0.42); guardGroup.add(sign);
  void door;
}
// end walls: mesh left / right of the belt opening and above it
const ox = LAYOUT.opening.x, [oz0, oz1] = LAYOUT.opening.z;
for (const sy of [-1, 1]) {
  const sideW = PX - 0.04 - ox;
  for (const sx of [-1, 1]) meshPanel(sideW, gh, sx * (ox + sideW / 2), sy * (PY + 0.05), gzc, 0);
  meshPanel(ox * 2, gz1 - oz1 - 0.01, 0, sy * (PY + 0.05), (oz1 + gz1) / 2 + 0.005, 0);
  for (const sx of [-1, 1]) beam80(V(sx * (ox + 0.04), sy * PY, gz0 - 0.08), V(sx * (ox + 0.04), sy * PY, TZ0), false);
  beam80(V(-ox, sy * PY, oz1 + 0.04), V(ox, sy * PY, oz1 + 0.04), false);
}
// light curtains at both belt openings (emitter / receiver), beams drawn faintly
const curtainBeams = [];
for (const sy of [-1, 1]) {
  for (const sx of [-1, 1]) {
    const x = sx * (ox - 0.02), y = sy * (PY + 0.10);
    rbox(0.034, 0.045, oz1 - oz0 + 0.06, 0.004, M.estopYellow, x, y, (oz0 + oz1) / 2, guardGroup);
    rbox(0.004, 0.03, oz1 - oz0, 0.001, M.blackPlastic, x - sx * 0.018, y, (oz0 + oz1) / 2, guardGroup);
    rbox(0.05, 0.05, 0.012, 0.003, M.powderBlack, x, y, oz0 - 0.03, guardGroup);
  }
  const bm = new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.18, depthWrite: false });
  for (let z = oz0 + 0.02; z < oz1; z += 0.03) { const o = new THREE.Mesh(new THREE.BoxGeometry(2 * ox - 0.08, 0.0012, 0.0012), bm); o.position.set(0, sy * (PY + 0.10), z); guardGroup.add(o); curtainBeams.push(o); }
}

// ---------------------------------------------------------------- electrical cabinet (RAL 7035) with main switch, and cable tray from the robot
const CAB = { x: -PX - 0.62, y: 0.15, w: 0.60, d: 0.40, h: 1.60 };
{
  const z0 = LAYOUT.floor + 0.10, g = new THREE.Group(); g.position.set(CAB.x, CAB.y, 0); world.add(g);
  rbox(CAB.d, CAB.w, 0.10, 0.004, M.powderBlack, 0, 0, LAYOUT.floor + 0.05, g);                         // plinth
  rbox(CAB.d, CAB.w, CAB.h, 0.008, M.ral7035, 0, 0, z0 + CAB.h / 2, g);
  rbox(0.012, CAB.w - 0.03, CAB.h - 0.03, 0.006, M.ral7035, -CAB.d / 2 - 0.006, 0, z0 + CAB.h / 2, g);    // door
  rbox(0.02, 0.03, 0.14, 0.006, M.blackPlastic, -CAB.d / 2 - 0.02, -CAB.w / 2 + 0.06, z0 + CAB.h * 0.55, g);  // handle
  for (const zz of [0.25, 1.35]) { rbox(0.006, 0.18, 0.12, 0.003, M.powderBlack, -CAB.d / 2 - 0.013, 0.12, z0 + zz, g); }   // filter fans
  rbox(0.006, 0.10, 0.10, 0.004, M.estopYellow, -CAB.d / 2 - 0.013, 0.15, z0 + CAB.h * 0.66, g);
  const ms = rbox(0.03, 0.022, 0.08, 0.006, M.estopRed, -CAB.d / 2 - 0.03, 0.15, z0 + CAB.h * 0.66, g); ms.rotation.x = 0.4;   // main switch
  const tag = labelPlane("=CAB1  400 V 3~  16 A", 0.18, 0.05, { bg: "#f3f3ee", fg: "#222", font: "600 64px 'IBM Plex Mono', monospace" });
  tag.rotation.set(Math.PI / 2, -Math.PI / 2, 0); tag.position.set(-CAB.d / 2 - 0.0125, -0.08, z0 + CAB.h - 0.15); g.add(tag);
  const warn = labelPlane("⚡", 0.06, 0.06, { bg: "#f2c400", fg: "#111", font: "700 180px sans-serif" });
  warn.rotation.set(Math.PI / 2, -Math.PI / 2, 0); warn.position.set(-CAB.d / 2 - 0.0125, -0.08, z0 + CAB.h - 0.27); g.add(warn);
}
// cable tray: on the -x carrier from the robot to the front cross beam, along it to the post, duct down to the cabinet roof
const trayMat = phys(0xb4b8bb, 0.45, 0.9);
function trayRun(a, b, w = 0.10) {
  const d = b.clone().sub(a), L = d.length(), g = new THREE.Group(); world.add(g);
  g.position.copy(a).addScaledVector(d, 0.5); g.quaternion.setFromUnitVectors(V(1, 0, 0), d.clone().normalize());
  rbox(L, w, 0.002, 0.0005, trayMat, 0, 0, 0, g);
  for (const s of [-1, 1]) rbox(L, 0.002, 0.05, 0.0005, trayMat, 0, s * w / 2, 0.025, g);
  return g;
}
trayRun(V(-LAYOUT.carriers, -0.05, TZ1 + 0.002), V(-LAYOUT.carriers, PY, TZ1 + 0.002));
trayRun(V(-LAYOUT.carriers, PY, TZ1 + 0.002), V(-PX, PY, TZ1 + 0.002));
const duct = rbox(0.08, 0.06, TZ1 - (LAYOUT.floor + 0.10 + CAB.h) + 0.06, 0.004, M.ral7035, -PX - 0.07, PY - 0.07, (TZ1 + LAYOUT.floor + 0.10 + CAB.h) / 2, world);
void duct;
tubeAlong([V(-PX - 0.07, PY - 0.07, LAYOUT.floor + 0.10 + CAB.h + 0.02), V(-PX - 0.25, PY - 0.2, LAYOUT.floor + 0.10 + CAB.h + 0.08), V(CAB.x + 0.05, CAB.y + 0.15, LAYOUT.floor + 0.10 + CAB.h + 0.03)], 0.012, M.blackRubber, world, 24, 10);
// motor cables from the head to the tray (drawn in 40-duopod); a bundle along the tray
tubeAlong([V(-LAYOUT.carriers + 0.02, -0.02, TZ1 + 0.012), V(-LAYOUT.carriers + 0.02, PY - 0.06, TZ1 + 0.012), V(-PX + 0.06, PY - 0.02, TZ1 + 0.012)], 0.008, M.blackRubber, world, 32, 8);

// ---------------------------------------------------------------- air preparation unit (filter-regulator, gauge, shut-off valve) on the -x / -y post
{
  const g = new THREE.Group(); g.position.set(-PX - 0.06, -PY + 0.0, 0.35); g.rotation.z = -Math.PI / 2; world.add(g);
  rbox(0.20, 0.008, 0.10, 0.002, M.galv, 0, 0, 0, g);
  rbox(0.05, 0.05, 0.06, 0.006, M.aluClear, -0.05, -0.03, 0.01, g);
  lathe([[0, -0.11], [0.018, -0.105], [0.022, -0.03], [0, -0.03]], M.pcClear, g).position.set(-0.05, -0.03, 0);
  rbox(0.05, 0.05, 0.06, 0.006, M.aluClear, 0.01, -0.03, 0.01, g);
  const gauge = vcyl(0.02, 0.012, M.chrome, 0.01, -0.06, 0.02, 24, g); gauge.rotation.set(0, 0, 0);
  const face = mesh(new THREE.CircleGeometry(0.017, 24), mat(0xf4f4f0, 0.6, 0), g); face.position.set(0.01, -0.0665, 0.02); face.rotation.x = Math.PI / 2;
  rbox(0.04, 0.05, 0.05, 0.006, M.puBlue, 0.07, -0.03, 0.01, g);
}

// ---------------------------------------------------------------- stack light on the front-right post
const stackLamps = { red: M.lampRed, amber: M.lampAmber, green: M.lampGreen };
function stackTurn(r, z0, z1, material) { const o = lathe([[0, z0], [r, z0], [r, z1], [0, z1]], material, world); o.position.set(LAYOUT.stack.x, LAYOUT.stack.y, 0); return o; }
{
  let z = TZ1;
  stackTurn(0.034, z, z + 0.035, M.blackPlastic); z += 0.035;
  stackTurn(0.0125, z, z + 0.150, M.blackPlastic); z += 0.150;
  stackTurn(0.030, z, z + 0.050, stackLamps.green); z += 0.050;
  stackTurn(0.030, z, z + 0.050, stackLamps.amber); z += 0.050;
  stackTurn(0.030, z, z + 0.050, stackLamps.red); z += 0.050;
  stackTurn(0.032, z, z + 0.028, M.blackPlastic);
}

// ---------------------------------------------------------------- HMI on a swing arm at the -x side, facing the walkway
const hmiRoot = new THREE.Group();
hmiRoot.position.set(LAYOUT.hmi.x - 0.20, LAYOUT.hmi.y - 0.25, LAYOUT.hmi.z + 0.55);
hmiRoot.rotation.z = Math.PI / 2 + 0.25;
world.add(hmiRoot);
rbox(0.41, 0.06, 0.30, 0.006, M.t3Dark, 0, 0, 0, hmiRoot);
const hmiCanvas = document.createElement("canvas"); hmiCanvas.width = 512; hmiCanvas.height = 384;
const hmiTexture = new THREE.CanvasTexture(hmiCanvas); hmiTexture.colorSpace = THREE.SRGBColorSpace;
const hmiScreen = { canvas: hmiCanvas, texture: hmiTexture };
const screenMat = M.screen.clone(); screenMat.map = hmiTexture; screenMat.emissiveMap = hmiTexture; screenMat.emissiveIntensity = 0.9;
const hmiFace = mesh(new THREE.PlaneGeometry(0.34, 0.255), screenMat, hmiRoot);
hmiFace.rotation.x = Math.PI / 2; hmiFace.position.y = 0.0305; hmiFace.castShadow = false;
{
  const arm = rod(0.018, M.aluClear, 16);
  placeRod(arm, V(-PX - 0.04, LAYOUT.hmi.y - 0.25, LAYOUT.hmi.z + 0.55), V(LAYOUT.hmi.x - 0.17, LAYOUT.hmi.y - 0.18, LAYOUT.hmi.z + 0.55));
  const es = lathe([[0, 0], [0.022, 0], [0.022, 0.012], [0.016, 0.022], [0, 0.024]], M.estopRed, hmiRoot);
  es.rotation.x = -Math.PI / 2; es.position.set(0.16, 0.03, -0.11);
  rbox(0.06, 0.012, 0.06, 0.003, M.estopYellow, 0.16, 0.031, -0.11, hmiRoot);
  for (const [x, col] of [[-0.16, M.lampGreen], [-0.12, M.lampAmber]]) { const b = lathe([[0, 0], [0.011, 0], [0.011, 0.008], [0, 0.008]], col, hmiRoot); b.rotation.x = -Math.PI / 2; b.position.set(x, 0.03, -0.115); }
}
