// Two belt conveyors along y, guides, fixed end stop, escapement (blade + side clamp), photo-eye. Plan §3.
const beltMats = {};
const DRUM_R = 0.040, BELT_T = 0.004;
// side-frame extrusion (60 x 110 conveyor profile with two outer T-slots), drawn in x/z and extruded along y
function sideFrameShape(s) {
  const pts = [[0, 0], [0.006, 0], [0.006, -0.110], [0, -0.110]];          // inner web
  const o = [[0.006, -0.002], [0.030, -0.002], [0.030, -0.030], [0.024, -0.030], [0.024, -0.038], [0.030, -0.038], [0.030, -0.072], [0.024, -0.072], [0.024, -0.080], [0.030, -0.080], [0.030, -0.108], [0.006, -0.108]];
  const outline = [[0, 0.004], ...o.slice(0, 1), ...o, [0, -0.110]];
  void pts;
  return outline.map(([x, z]) => [s * x, z]);
}
for (const [name, B] of Object.entries(BELTS)) {
  const L = B.y1 - B.y0, yc = (B.y0 + B.y1) / 2, bw = B.w + 0.04, xo = Math.sign(B.x);
  const m = M.beltGreen.clone(); m.map = M.beltGreen.map.clone(); m.map.repeat.set(2, L / 0.16); m.map.needsUpdate = true;
  beltMats[name] = m;
  rbox(bw, L, BELT_T, 0.0015, m, B.x, yc, -BELT_T / 2, world);                                       // top run
  rbox(bw, L, BELT_T, 0.0015, M.beltDark, B.x, yc, -2 * DRUM_R - BELT_T / 2 + 0.0, world);            // return run
  for (const y of [B.y0, B.y1]) {
    const wrap = mesh(new THREE.CylinderGeometry(DRUM_R + BELT_T, DRUM_R + BELT_T, bw, 40, 1, true, y === B.y0 ? Math.PI : 0, Math.PI), M.beltDark);
    wrap.rotation.set(0, 0, Math.PI / 2); wrap.position.set(B.x, y, -DRUM_R);
    const d = vcyl(DRUM_R, bw + 0.002, M.steel, B.x, y, -DRUM_R, 40); d.rotation.set(0, 0, Math.PI / 2);
    for (const sx of [-1, 1]) {   // pillow-block bearings on the outer faces
      rbox(0.022, 0.07, 0.05, 0.005, M.powderBlack, B.x + sx * (bw / 2 + 0.042), y, -DRUM_R, world);
      const shaft = vcyl(0.010, 0.03, M.steel, B.x + sx * (bw / 2 + 0.05), y, -DRUM_R, 16); shaft.rotation.set(0, 0, Math.PI / 2);
    }
  }
  for (let y = B.y0 + 0.4; y < B.y1 - 0.2; y += 0.6) { const r = vcyl(0.016, bw - 0.01, M.galv, B.x, y, -2 * DRUM_R - 0.022, 20); r.rotation.set(0, 0, Math.PI / 2); }   // return rollers
  for (const sd of [-1, 1]) {
    const xs = B.x + sd * (bw / 2 + 0.002);
    extrudeXZ(sideFrameShape(sd), B.y0 - 0.06, B.y1 + 0.06, M.aluClear, world, 0.001).position.x = xs;
    for (let y = B.y0 + 0.15; y < B.y1; y += 0.40) addBolt(V(xs + sd * 0.030, y, -0.020), V(sd, 0, 0));
    // guide: HDPE wear strip on an aluminium carrier, adjustable brackets with star knobs
    const xg = B.x + sd * (B.w / 2 + GUIDE.t / 2);
    rbox(GUIDE.t, L, GUIDE.top - 0.010, 0.002, M.guideHDPE, xg, yc, (GUIDE.top + 0.010) / 2, world);
    rbox(0.006, L, 0.030, 0.0015, M.aluClear, xg + sd * (GUIDE.t / 2 + 0.003), yc, 0.038, world);
    for (let y = B.y0 + 0.15; y < B.y1 - 0.05; y += 0.45) {
      const post = rod(0.006, M.steel, 12); placeRod(post, V(xs + sd * 0.018, y, -0.004), V(xs + sd * 0.018, y, 0.045));
      rbox(0.022, 0.022, 0.018, 0.003, M.blackPlastic, xs + sd * 0.018, y, 0.040, world);
      const bar = rod(0.005, M.steel, 12); placeRod(bar, V(xs + sd * 0.018, y, 0.040), V(xg + sd * GUIDE.t, y, 0.040));
      const knob = lathe([[0, 0], [0.013, 0], [0.013, 0.010], [0.007, 0.014], [0, 0.014]], M.blackPlastic, world); knob.position.set(xs + sd * 0.018, y, 0.049);
    }
  }
  // legs (40 x 40 with a cross brace), feet
  const nLeg = Math.ceil((L - 0.24) / 0.9), step = (L - 0.24) / nLeg;
  for (let k = 0; k <= nLeg; k++) {
    const y = B.y0 + 0.12 + k * step;
    for (const sd of [-1, 1]) {
      const x = B.x + sd * (bw / 2 + 0.02);
      rbox(0.04, 0.04, -0.11 - LAYOUT.floor - 0.03, 0.003, M.aluClear, x, y, (LAYOUT.floor + 0.03 - 0.11) / 2, world);
      lathe([[0, LAYOUT.floor], [0.03, LAYOUT.floor], [0.03, LAYOUT.floor + 0.012], [0.008, LAYOUT.floor + 0.03], [0, LAYOUT.floor + 0.03]], M.blackRubber, world).position.set(x, y, 0);
    }
    rbox(bw + 0.04, 0.03, 0.03, 0.003, M.aluClear, B.x, y, -0.40, world);
    const br = rod(0.008, M.aluClear, 8); placeRod(br, V(B.x - bw / 2 - 0.02, y, -0.40), V(B.x + bw / 2 + 0.02, y, -0.12));
  }
  // drive: helical geared motor on the outer side at the downstream end, terminal box, shaft guard
  const yd = B.y1, xm = B.x + xo * (bw / 2 + 0.03);
  const gb = rbox(0.09, 0.11, 0.12, 0.012, M.zinc9006, xm + xo * 0.06, yd, -DRUM_R - 0.01, world); void gb;
  const mot = vcyl(0.050, 0.16, M.ral7035, xm + xo * 0.19, yd, -DRUM_R, 40); mot.rotation.set(0, 0, Math.PI / 2);
  const fan = vcyl(0.052, 0.04, M.powderBlack, xm + xo * 0.29, yd, -DRUM_R, 40); fan.rotation.set(0, 0, Math.PI / 2);
  rbox(0.07, 0.07, 0.04, 0.006, M.ral7035, xm + xo * 0.19, yd, -DRUM_R + 0.06, world);
  const ce = labelPlane(name === "A" ? "M3 · belt A" : "M4 · belt B", 0.07, 0.02, { bg: "#f3f3ee", fg: "#222", font: "600 64px 'IBM Plex Mono', monospace" });
  ce.position.set(xm + xo * 0.19, yd, -DRUM_R + 0.0805); world.add(ce);
}

// ---------------------------------------------------------------- pneumatics: compact cylinder helper, valve terminal, PU tubes
function compactCylinder(parent, stroke = 0.12) {   // body along +x from the origin, rod end at x = 0
  const g = new THREE.Group(); parent.add(g);
  rbox(0.10, 0.045, 0.045, 0.004, M.cylAlu, 0.05 + stroke * 0, 0, 0, g);
  for (const x of [0.004, 0.096]) rbox(0.008, 0.05, 0.05, 0.004, M.aluDark, x, 0, 0, g);
  for (const x of [0.02, 0.08]) { const f = vcyl(0.004, 0.012, M.zinc9006, x, 0, 0.028, 12, g); f.rotation.set(0, 0, 0); }
  return g;
}
const valveTerminal = new THREE.Group(); world.add(valveTerminal);
{
  const A = BELTS.A, so0 = Math.sign(A.x), x = A.x + so0 * (A.w / 2 + 0.12), y = BLADE_Y - 0.36;
  valveTerminal.position.set(x, y, -0.05);
  rbox(0.05, 0.16, 0.05, 0.004, M.powderBlack, 0, 0, 0, valveTerminal);
  for (let k = 0; k < 3; k++) { rbox(0.03, 0.035, 0.07, 0.003, M.siliconeBlue, 0, -0.05 + k * 0.05, 0.055, valveTerminal); rbox(0.012, 0.02, 0.012, 0.002, M.lampAmber, 0.016, -0.05 + k * 0.05, 0.08, valveTerminal); }
}
// ---------------------------------------------------------------- fixed end stop on belt A (carton front edge at STOP_Y)
{
  const A = BELTS.A;
  rbox(A.w, 0.012, 0.066, 0.002, M.steel, A.x, STOP_Y + 0.006, 0.004 + 0.033, world);
  rbox(A.w - 0.02, 0.004, 0.050, 0.001, M.blackRubber, A.x, STOP_Y + 0.002, 0.040, world);
  for (const s of [-1, 1]) rbox(0.012, 0.05, 0.10, 0.002, M.steel, A.x + s * (A.w / 2 + 0.016), STOP_Y + 0.02, 0.02, world);
}
// photo-eye (retro-reflective) across belt A just before the stop; so = outer side of belt A (+1 or -1 in x)
const so = Math.sign(BELTS.A.x), aIn = (d) => BELTS.A.x - so * d, aOut = (d) => BELTS.A.x + so * d;
const eyeY = STOP_Y - 0.04;
rbox(0.030, 0.020, 0.040, 0.003, M.sensorBlue, aOut(BELTS.A.w / 2 + 0.030), eyeY, 0.07, world);
rbox(0.006, 0.030, 0.030, 0.002, M.lampRed, aIn(BELTS.A.w / 2 + 0.016), eyeY, 0.07, world);
const eyeBeam = new THREE.Mesh(new THREE.CylinderGeometry(0.0015, 0.0015, BELTS.A.w + 0.02, 6), new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.55 }));
eyeBeam.rotation.z = Math.PI / 2; eyeBeam.position.set(BELTS.A.x, eyeY, 0.07); world.add(eyeBeam);

// ---------------------------------------------------------------- escapement on the outer side of belt A: blade at BLADE_Y, side clamp one carton upstream
const escBlade = new THREE.Group(); world.add(escBlade);
const bladeRoot = aOut(BELTS.A.w / 2 + 0.10);
{
  const cg = new THREE.Group(); cg.position.set(aOut(BELTS.A.w / 2 + 0.14), BLADE_Y - 0.004, 0.035); cg.rotation.z = so > 0 ? 0 : Math.PI; world.add(cg);
  compactCylinder(cg); cg.children[0].position.x = -0.03;
  rbox(0.02, 0.06, 0.06, 0.003, M.galv, aOut(BELTS.A.w / 2 + 0.10), BLADE_Y - 0.004, 0.0, world);             // bracket to the frame
  rbox(0.20, 0.008, 0.040, 0.002, M.estopYellow, -so * 0.10, 0, 0, escBlade);                                   // blade, origin at its root
  rbox(0.012, 0.014, 0.014, 0.002, M.steel, so * 0.02, 0, 0, escBlade);                                         // piston rod clevis
  const pr = rod(0.005, M.chrome, 12); escBlade.add(pr); placeRod(pr, V(so * 0.01, 0, 0), V(so * 0.075, 0, 0));
  escBlade.position.set(bladeRoot, BLADE_Y - 0.004, 0.035);
}
const escClamp = new THREE.Group(); world.add(escClamp);
// top clamp: a vertical compact cylinder on a bracket over belt A presses the queued carton from above (no sideways push;
// the MuJoCo run showed a side clamp shoves the carton against the opposite guide and it rubs the guide when lifted)
const clampY = BLADE_Y - 1.5 * CARTON.y, CLAMP_Z0 = CARTON.h + 0.028;
{
  const A = BELTS.A, xo = aOut(A.w / 2 + 0.05);
  rbox(0.03, 0.04, CLAMP_Z0 + 0.16, 0.003, M.galv, xo, clampY, (CLAMP_Z0 + 0.16) / 2 - 0.02, world);             // upright on the frame
  rbox(Math.abs(xo - A.x) + 0.04, 0.04, 0.02, 0.003, M.galv, (xo + A.x) / 2, clampY, CLAMP_Z0 + 0.15, world);       // arm over the belt
  const cg = new THREE.Group(); cg.position.set(A.x, clampY, CLAMP_Z0 + 0.13); cg.rotation.y = Math.PI / 2; world.add(cg);
  compactCylinder(cg); cg.children[0].position.x = -0.0;
  rbox(0.12, 0.10, 0.012, 0.003, M.blackRubber, 0, 0, 0, escClamp);                                                 // pad
  const pr = rod(0.005, M.chrome, 12); escClamp.add(pr); placeRod(pr, V(0, 0, 0.006), V(0, 0, 0.05));
  escClamp.position.set(A.x, clampY, CLAMP_Z0);
}
// PU tubes from the valve terminal to both cylinders
{
  const vt = valveTerminal.position, A = BELTS.A;
  for (const [k, yT] of [[0, BLADE_Y - 0.004], [1, clampY]]) {
    const xT = k ? A.x : aOut(A.w / 2 + 0.17), zT = k ? CLAMP_Z0 + 0.22 : 0.065, y0 = vt.y - 0.05 + k * 0.05;
    tubeAlong([V(vt.x, y0, vt.z + 0.09), V(vt.x, y0 + 0.02, 0.10), V((vt.x + xT) / 2, (y0 + yT) / 2, k ? 0.32 : 0.11), V(xT, yT - 0.0, zT)], 0.003, M.puBlueTube, world, 32, 8);
  }
  tubeAlong([V(vt.x, vt.y + 0.08, vt.z), V(vt.x + Math.sign(A.x) * 0.1, vt.y - 0.2, -0.3), V(PX + 0.0, -PY + 0.05, -0.35)], 0.004, M.puBlueTube, world, 32, 8);
}
function escapeState(t) {
  let open = false, clampOn = false;
  for (const e of escape) {
    if (e.t > t) break;
    if (e.ev === "release") { open = true; clampOn = e.clamp >= 0; }
    else { open = false; clampOn = false; }
  }
  return { open, clampOn };
}
function setConveyors(t) {
  for (const [name, B] of Object.entries(BELTS)) {
    const m = beltMats[name]; m.map.offset.y = -(t * B.v) / 0.16;
  }
  const st = escapeState(t);
  if (PHYS) {          // physics replay: actual slide travel of the blade and the clamp
    const i = physIndex(t)[0];
    escBlade.position.x = bladeRoot + so * PHYS.blade[i];
    escClamp.position.z = CLAMP_Z0 - PHYS.clamp[i];
  } else {
    escBlade.position.x = bladeRoot + (st.open ? so * 0.12 : 0);
    escClamp.position.z = CLAMP_Z0 - (st.clampOn ? 0.022 : 0);
  }
  return st;
}
