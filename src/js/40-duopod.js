// ---------------------------------------------------------------- DuoPod RVS2-1000-3kg: procedural model on the STEP joint centres, optional vendor GLB
// Robot frame = world translated to the motor axes (z = ROBOT.zm). The vendor CAD maps as x_r = y_cad, y_r = 0.065 - x_cad, z_r = z_cad.
// Every moving part is a group whose matrix moves it from the CAD reference pose to the current pose.
const robotRoot = new THREE.Group(); robotRoot.position.set(0, 0, ROBOT.zm); world.add(robotRoot);
const REF = (() => {
  const A = [[-ROBOT.b, 0], [ROBOT.b, 0]], E = [[-ROBOT.ref.elbow[0], ROBOT.ref.elbow[1]], [ROBOT.ref.elbow[0], ROBOT.ref.elbow[1]]];
  const J = [[-ROBOT.e, -0.705], [ROBOT.e, -0.705]], F = [0, -0.744];
  return { A, E, J, F };
})();
const PART_NAMES = ["head", "upperL", "upperR", "lowerL", "lowerR", "plate", "tri", "rod1", "rod2"];
const parts = Object.fromEntries(PART_NAMES.map((n) => {
  const g = new THREE.Group(); g.matrixAutoUpdate = false; robotRoot.add(g);
  const proc = new THREE.Group(); g.add(proc);
  return [n, { g, proc }];
}));
const vY = (x, z, y = 0) => V(x, y, z);
const ang = (a, b) => Math.atan2(b[1] - a[1], b[0] - a[0]);
function yBar(r, y0, y1, m, x, z, parent, seg = 32) {      // cylinder along y in a plane point
  const o = mesh(new THREE.CylinderGeometry(r, r, y1 - y0, seg), m, parent); o.position.set(x, (y0 + y1) / 2, z); return o;
}
function tubeXZ(r, a, b, y, m, parent, seg = 24) { const o = rod(r, m, seg); parent.add(o); placeRod(o, vY(a[0], a[1], y), vY(b[0], b[1], y)); return o; }
function hullOfCircles(cs, r, n = 20) {     // rounded convex outline around circle centres [[x, z], ...]
  const pts = [];
  for (const c of cs) for (let i = 0; i < n; i++) { const a = i / n * Math.PI * 2; pts.push([c[0] + r * Math.cos(a), c[1] + r * Math.sin(a)]); }
  pts.sort((p, q) => p[0] - q[0] || p[1] - q[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], hi = [];
  for (const p of pts) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (const p of pts.slice().reverse()) { while (hi.length >= 2 && cross(hi[hi.length - 2], hi[hi.length - 1], p) <= 0) hi.pop(); hi.push(p); }
  return lo.slice(0, -1).concat(hi.slice(0, -1));
}

// ---- hi-fi helpers: outlines with holes extruded along y, keeping the outer size exact (bevel pulled inwards)
function shapeOf(outer, holes = []) {
  const sh = new THREE.Shape(outer.map(([x, z]) => new THREE.Vector2(x, z)));
  for (const h of holes) sh.holes.push(new THREE.Path(h.map(([x, z]) => new THREE.Vector2(x, z))));
  return sh;
}
function extrudeXZShape(shape, y0, y1, m, parent, b = 0.0015) {
  const t = y1 - y0, bb = Math.min(b, t / 3);
  const geo = new THREE.ExtrudeGeometry(shape, { depth: t - 2 * bb, bevelEnabled: true, bevelThickness: bb, bevelSize: bb, bevelOffset: -bb, bevelSegments: 2, curveSegments: 24 });
  geo.translate(0, 0, bb); geo.rotateX(Math.PI / 2); geo.translate(0, y1, 0);
  return mesh(geo, m, parent);
}
const circlePts = (cx, cz, r, n = 28, rev = false) => Array.from({ length: n }, (_, i) => { const a = (rev ? -1 : 1) * i / n * Math.PI * 2; return [cx + r * Math.cos(a), cz + r * Math.sin(a)]; });
function insetHull(cs, r) { return hullOfCircles(cs, r, 16); }
// bolt circle on a face normal to +/-y
function boltRing(cx, cz, y, r, n, parent, dir = 1) { for (let i = 0; i < n; i++) { const a = (i + 0.5) / n * Math.PI * 2; addBolt(V(cx + r * Math.cos(a), y, cz + r * Math.sin(a)), V(0, dir, 0), parent); } }
// lathe about the y axis at (x, z): profile [[r, y], ...]
function latheY(prof, m, x, z, parent, seg = 48) { const geo = new THREE.LatheGeometry(prof.map(([r, y]) => new THREE.Vector2(r, y)), seg); const o = mesh(geo, m, parent); o.position.set(x, 0, z); return o; }

// ---- head (fixed): machined body, mounting plate with the drawing's hole pattern, gearboxes with output flanges
{
  const P = parts.head.proc, bX = ROBOT.b;
  // mounting plate 430 x 290 x 30, centre window, fastening holes (connecting dimensions p.2: x ±200/±170/±135)
  const plate = [[-0.215, -0.145], [0.215, -0.145], [0.215, 0.145], [-0.215, 0.145]];
  const holes = [[-0.08, -0.05], [0.08, -0.05], [0.08, 0.05], [-0.08, 0.05]].map(([x, y]) => [x, y]);
  const mp = new THREE.Shape(plate.map(([x, y]) => new THREE.Vector2(x, y)));
  mp.holes.push(new THREE.Path([[-0.075, -0.045], [-0.075, 0.045], [0.075, 0.045], [0.075, -0.045]].map(([x, y]) => new THREE.Vector2(x, y))));
  for (const [x, y] of [[-0.2, 0], [0.2, 0], [-0.17, -0.105], [0.17, -0.105], [-0.17, 0.105], [0.17, 0.105], [-0.135, -0.125], [0.135, -0.125], [-0.135, 0.12], [0.135, 0.12]])
    mp.holes.push(new THREE.Path(circlePts(x, y, 0.011, 20, true).map(([a, b]) => new THREE.Vector2(a, b))));
  const mpg = new THREE.ExtrudeGeometry(mp, { depth: 0.026, bevelEnabled: true, bevelThickness: 0.002, bevelSize: 0.002, bevelOffset: -0.002, bevelSegments: 2, curveSegments: 20 });
  mpg.translate(0, 0, 0.142); mesh(mpg, M.headAlu, P);
  void holes;
  // front and rear walls: lobes around the gearbox flanges (hull of the two flange circles and the top corners)
  const wall = hullOfCircles([[-bX, 0], [bX, 0]], 0.094, 32).concat([[0.215, 0.142], [-0.215, 0.142]]);
  const wallHull = hullOfCircles([[-bX, 0], [bX, 0], [-0.19, 0.118], [0.19, 0.118]], 0.024, 24);
  void wall;
  for (const s of [-1, 1]) {
    const bores = [[-bX, 0], [bX, 0]].map(([x, z]) => circlePts(x, z, 0.060, 40, true));
    extrudeXZShape(shapeOf(wallHull, s > 0 ? bores : []), s > 0 ? 0.115 : -0.145, s > 0 ? 0.145 : -0.115, M.headAlu, P, 0.002);
    // end walls with a machined pocket
    rbox(0.026, 0.260, 0.215, 0.004, M.headAlu, s * 0.202, 0, 0.034, P);
    rbox(0.004, 0.200, 0.150, 0.002, M.aluBlasted, s * 0.216, 0, 0.040, P);
    yBar(0.0055, -0.11, 0.11, M.steel, s * (bX + ROBOT.o1[0]), ROBOT.o1[1], P, 16);   // rod-1 pivot pin (right side used)
  }
  // rear wall bearing covers
  for (const s of [-1, 1]) { latheY([[0, -0.152], [0.05, -0.152], [0.056, -0.147], [0.056, -0.145]], M.aluClear, s * bX, 0, P); boltRing(s * bX, 0, -0.152, 0.045, 8, P, -1); }
  // gearboxes: flange ring, housing, output dome, bolt circles
  for (const s of [-1, 1]) {
    latheY([[0.058, 0.145], [0.082, 0.145], [0.082, 0.155], [0.075, 0.158], [0.072, 0.197], [0.064, 0.200], [0.058, 0.200]], M.gearboxSilver, s * bX, 0, P, 64);
    latheY([[0.050, 0.145], [0.058, 0.145], [0.058, 0.200], [0.050, 0.200]], M.aluDark, s * bX, 0, P, 48);
    boltRing(s * bX, 0, 0.158, 0.077, 16, P, 1);
  }
  // type plate on the front wall between the gearboxes (generic, no brand)
  const tag = labelPlane("2-AXIS PARALLEL · 3 kg", 0.08, 0.03, { bg: "#d9dde1", fg: "#222", font: "600 64px 'IBM Plex Sans', sans-serif" });
  tag.rotation.x = Math.PI / 2; tag.rotation.y = Math.PI; tag.position.set(0, 0.1465, 0.075); P.add(tag);
}
// ---- servo motors (not in the vendor CAD): F5 flange 104 mm, black housing, angled one-cable connector, cable to the tray
const motors = new THREE.Group(); robotRoot.add(motors);
const motorCables = [];
for (const s of [-1, 1]) {
  const x = s * ROBOT.b;
  rbox(0.112, 0.028, 0.112, 0.006, M.aluClear, x, 0.214, 0, motors);                 // servo adapter
  for (const [dx, dz] of [[0.042, 0.042], [-0.042, 0.042], [0.042, -0.042], [-0.042, -0.042]]) addBolt(V(x + dx, 0.228, dz), V(0, 1, 0), motors);
  rbox(0.104, 0.016, 0.104, 0.008, M.motorBlack, x, 0.236, 0, motors);                // motor flange
  rbox(0.098, 0.150, 0.098, 0.016, M.motorBlack, x, 0.318, 0, motors);                // housing
  rbox(0.090, 0.030, 0.090, 0.018, M.blackPlastic, x, 0.405, 0, motors);              // feedback cover
  rbox(0.100, 0.004, 0.100, 0.016, M.aluDark, x, 0.392, 0, motors);                  // housing / cover joint
  const con = new THREE.Group(); con.position.set(x, 0.372, 0.052); con.rotation.x = -0.6; motors.add(con);
  rbox(0.034, 0.034, 0.030, 0.004, M.blackPlastic, 0, 0, 0.012, con);
  const nut = vcyl(0.014, 0.022, M.zinc9006, 0, 0, 0.036, 20, con);
  const lab = labelPlane(s < 0 ? "M2" : "M1", 0.044, 0.022, { bg: "#18191b", fg: "#e8e8e8" });
  lab.rotation.x = Math.PI / 2; lab.rotation.y = Math.PI; lab.position.set(x, 0.330, -0.0495); motors.add(lab);
  void nut;
  motorCables.push(tubeAlong([V(x, 0.395, 0.085), V(x, 0.42, 0.13), V(x * 0.7, 0.36, 0.175), V(x * 0.4, 0.25, 0.19), V(s * 0.30, 0.20, 0.215)], 0.0075, M.blackRubber, robotRoot, 40, 10));
}

// ---- upper arms: clamping hub on the gearbox output, carbon tube with bonded sleeves, elbow bearing housing
[["upperL", 0], ["upperR", 1]].forEach(([n, i]) => {
  const P = parts[n].proc, A = REF.A[i], E = REF.E[i];
  const d = [E[0] - A[0], E[1] - A[1]], L = Math.hypot(...d), u = [d[0] / L, d[1] / L];
  latheY([[0.030, -0.056], [0.056, -0.056], [0.060, -0.050], [0.060, 0.050], [0.056, 0.056], [0.030, 0.056]], M.aluClear, A[0], A[1], P, 56);
  boltRing(A[0], A[1], 0.056, 0.044, 8, P, 1);
  const s0 = [A[0] + u[0] * 0.05, A[1] + u[1] * 0.05], s1 = [A[0] + u[0] * 0.115, A[1] + u[1] * 0.115];
  tubeXZ(0.047, s0, s1, 0, M.aluClear, P, 40);                                  // hub sleeve
  const e0 = [E[0] - u[0] * 0.105, E[1] - u[1] * 0.105], e1 = [E[0] - u[0] * 0.035, E[1] - u[1] * 0.035];
  tubeXZ(0.042, s1, e0, 0, M.carbonGloss, P, 40);
  tubeXZ(0.045, e0, e1, 0, M.aluClear, P, 40);                                  // elbow sleeve
  for (const q of [s1, e0]) { const r = rod(0.0465, M.aluDark, 40); P.add(r); placeRod(r, V(q[0] - u[0] * 0.002, 0, q[1] - u[1] * 0.002), V(q[0] + u[0] * 0.002, 0, q[1] + u[1] * 0.002)); }
  latheY([[0.022, -0.056], [0.045, -0.056], [0.049, -0.051], [0.049, 0.051], [0.045, 0.056], [0.022, 0.056]], M.aluClear, E[0], E[1], P, 48);
  latheY([[0, -0.060], [0.024, -0.060], [0.024, 0.060], [0, 0.060]], M.steel, E[0], E[1], P, 24);
  for (const y of [-0.0565, 0.0565]) latheY([[0.026, y - 0.0015], [0.040, y - 0.0015], [0.040, y + 0.0015], [0.026, y + 0.0015]], M.blackRubber, E[0], E[1], P, 40);
});
// ---- lower arms: carbon tube, tapered aluminium end fittings, fork eyes with bearing housings
[["lowerL", 0], ["lowerR", 1]].forEach(([n, i]) => {
  const P = parts[n].proc, E = REF.E[i], J = REF.J[i];
  const d = [J[0] - E[0], J[1] - E[1]], L = Math.hypot(...d), u = [d[0] / L, d[1] / L];
  const at = (k) => [E[0] + u[0] * k, E[1] + u[1] * k];
  tubeXZ(0.032, at(0.12), at(L - 0.12), 0, M.carbonGloss, P, 40);
  for (const [k0, k1] of [[0.035, 0.12], [L - 0.12, L - 0.035]]) {
    const r = mesh(new THREE.CylinderGeometry(k0 < 0.1 ? 0.033 : 0.024, k0 < 0.1 ? 0.024 : 0.033, k1 - k0, 32), M.aluClear, P);
    const a = at(k0), b = at(k1); r.position.set((a[0] + b[0]) / 2, 0, (a[1] + b[1]) / 2); r.quaternion.setFromUnitVectors(UP, V(u[0], 0, u[1]));
  }
  for (const [q, side] of [[E, 1], [J, -1]]) {
    for (const y of [-0.030, 0.030]) { const lug = hullOfCircles([q, [q[0] + side * u[0] * 0.05, q[1] + side * u[1] * 0.05]], 0.029, 20); extrudeXZShape(shapeOf(lug), y - 0.007, y + 0.007, M.aluClear, P, 0.0015); }
    latheY([[0.012, -0.037], [0.022, -0.037], [0.022, 0.037], [0.012, 0.037]], M.steel, q[0], q[1], P, 24);
  }
});
// ---- moving plate (red): two machined cheeks with pockets, bridge, ISO-style flange (4 x M8 at ±60 / ±42.5, Ø45 bore, Ø6 pin)
const PLATE_PIVOT = [ROBOT.e + ROBOT.o2[0], -0.705 + ROBOT.o2[1]];
{
  const P = parts.plate.proc;
  const outline = hullOfCircles([REF.J[0], REF.J[1], PLATE_PIVOT], 0.039, 28);
  const pocket = hullOfCircles([[-0.035, -0.712], [0.035, -0.712], [0.0, -0.684]], 0.010, 12).reverse();
  const bores = REF.J.map(([x, z]) => circlePts(x, z, 0.020, 28, true));
  for (const s of [-1, 1]) extrudeXZShape(shapeOf(outline, [pocket, ...bores]), s > 0 ? 0.045 : -0.062, s > 0 ? 0.062 : -0.045, M.plateRed, P, 0.0025);
  rbox(0.240, 0.090, 0.022, 0.004, M.plateRed, 0, 0, -0.744 + 0.011, P);          // bridge, bottom = flange plane
  for (const J of REF.J) {
    latheY([[0.014, -0.068], [0.026, -0.068], [0.026, 0.068], [0.014, 0.068]], M.aluClear, J[0], J[1], P, 32);
    for (const y of [-0.066, 0.066]) latheY([[0.026, y - 0.002], [0.034, y - 0.002], [0.034, y + 0.002], [0.026, y + 0.002]], M.blackRubber, J[0], J[1], P, 32);
  }
  latheY([[0, -0.077], [0.006, -0.077], [0.006, 0.077], [0, 0.077]], M.steel, PLATE_PIVOT[0], PLATE_PIVOT[1], P, 16);
}
// ---- gripper (also shown with the vendor CAD): adapter on the ISO flange, ejector with silencer and LED, manifold, foam pad
{
  const zf = -0.744, P2 = new THREE.Group(); parts.plate.g.add(P2);
  rbox(0.140, 0.100, 0.014, 0.003, M.aluClear, 0, 0, zf - 0.007, P2);
  for (const [x, y] of [[0.06, 0.0425], [-0.06, 0.0425], [0.06, -0.0425], [-0.06, -0.0425]]) addBolt(V(x, y, zf - 0.014), V(0, 0, -1), P2);
  rbox(0.100, 0.064, 0.042, 0.004, M.aluBlasted, 0, 0, zf - 0.014 - 0.021, P2);
  rbox(0.046, 0.034, 0.024, 0.003, M.siliconeBlue, 0.074, 0, zf - 0.034, P2);       // ejector
  const sil = vcyl(0.008, 0.03, M.blackPlastic, 0.106, 0, zf - 0.034, 16, P2); sil.rotation.set(0, 0, Math.PI / 2);
  const fit = vcyl(0.006, 0.016, M.zinc9006, 0.074, -0.014, zf - 0.018, 12, P2); fit.rotation.set(0, 0, 0);
  rbox(TOOL.padX, TOOL.padY, 0.022, 0.004, M.aluClear, 0, 0, zf - 0.056 - 0.011, P2);
  for (const [x, y] of [[0.06, 0.09], [-0.06, 0.09], [0.06, -0.09], [-0.06, -0.09], [0.06, 0], [-0.06, 0]]) addBolt(V(x, y, zf - 0.056), V(0, 0, 1), P2);
  rbox(TOOL.padX - 0.002, TOOL.padY - 0.002, 0.032, 0.006, M.foam, 0, 0, zf - TOOL.len + 0.016, P2);
  // foam suction holes (dark dots on the face)
  const dot = new THREE.CircleGeometry(0.0045, 12), dm = M.blackRubber;
  for (let x = -0.06; x <= 0.061; x += 0.03) for (let y = -0.09; y <= 0.091; y += 0.03) { const o = mesh(dot, dm, P2); o.rotation.x = Math.PI; o.position.set(x, y, zf - TOOL.len - 0.0003); o.castShadow = false; }
}
// ---- orientation linkage: triangle with lightening cut-outs, rods with ball sockets
const TRI = { E: REF.E[1], top: [REF.E[1][0] + ROBOT.o1[0], REF.E[1][1] + ROBOT.o1[1]], low: [REF.E[1][0] + ROBOT.o2[0], REF.E[1][1] + ROBOT.o2[1]] };
{
  const P = parts.tri.proc, outline = hullOfCircles([TRI.E, TRI.top, TRI.low], 0.026, 24);
  const c = [(TRI.E[0] + TRI.top[0] + TRI.low[0]) / 3, (TRI.E[1] + TRI.top[1] + TRI.low[1]) / 3];
  const toward = (p, k) => [p[0] + (c[0] - p[0]) * k, p[1] + (c[1] - p[1]) * k];
  const cut = hullOfCircles([toward(TRI.E, 0.55), toward(TRI.top, 0.55), toward(TRI.low, 0.5)], 0.008, 12).reverse();
  for (const s of [-1, 1]) extrudeXZShape(shapeOf(outline, [cut]), s > 0 ? 0.078 : -0.091, s > 0 ? 0.091 : -0.078, M.aluBlasted, P, 0.002);
  for (const q of [TRI.top, TRI.low]) latheY([[0, -0.106], [0.0058, -0.106], [0.0058, 0.106], [0, 0.106]], M.steel, q[0], q[1], P, 16);
  const B = [ROBOT.b + ROBOT.o1[0], ROBOT.o1[1]];
  const rodPair = (a, b, ys, r, parent) => {
    const d = [b[0] - a[0], b[1] - a[1]], L = Math.hypot(...d), u = [d[0] / L, d[1] / L];
    for (const y of ys) {
      tubeXZ(r, [a[0] + u[0] * 0.035, a[1] + u[1] * 0.035], [b[0] - u[0] * 0.035, b[1] - u[1] * 0.035], y, M.carbonMatte, parent, 16);
      for (const [q, sg] of [[a, 1], [b, -1]]) {
        tubeXZ(r + 0.003, [q[0] + sg * u[0] * 0.012, q[1] + sg * u[1] * 0.012], [q[0] + sg * u[0] * 0.04, q[1] + sg * u[1] * 0.04], y, M.aluClear, parent, 16);
        const cup = mesh(new THREE.SphereGeometry(0.0135, 20, 14, 0, Math.PI * 2, 0, Math.PI * 0.62), M.aluClear, parent);
        cup.position.set(q[0], y, q[1]); cup.quaternion.setFromUnitVectors(UP, V(sg * u[0], 0, sg * u[1]));
        const ball = mesh(new THREE.SphereGeometry(0.010, 16, 12), M.chrome, parent); ball.position.set(q[0], y, q[1]);
      }
    }
  };
  rodPair(B, TRI.top, [-0.1015, 0.1015], 0.0075, parts.rod1.proc);
  rodPair(TRI.low, PLATE_PIVOT, [-0.071, 0.071], 0.0068, parts.rod2.proc);
}
// vacuum hose: ejector -> along the left lower arm -> head
const hose = tubeAlong([[0, 0, 0], [0, 0, 1]], 0.006, M.blackRubber, robotRoot, 48, 8);

// ---- pose: matrices from the CAD reference pose to the current pose
const _m1 = new THREE.Matrix4(), _m2 = new THREE.Matrix4(), _m3 = new THREE.Matrix4();
function pivotRot(g, pivotRef, pivotNew, dAng) {   // T(new) * Ry(-d) * T(-ref)
  g.matrix.makeTranslation(pivotNew[0], 0, pivotNew[1]).multiply(_m2.makeRotationY(-dAng)).multiply(_m3.makeTranslation(-pivotRef[0], 0, -pivotRef[1]));
}
let robotPose = null;
function setRobot(tcp) {
  const pose = armPose(tcp); if (!pose) return null;
  robotPose = pose;
  const rel = (p) => [p[0], p[1] - ROBOT.zm];
  const A = pose.motors.map(rel), E = pose.elbows.map(rel), J = pose.joints.map(rel), F = rel(pose.flange);
  parts.head.g.matrix.identity();
  ["upperL", "upperR"].forEach((n, i) => pivotRot(parts[n].g, REF.A[i], A[i], ang(A[i], E[i]) - ang(REF.A[i], REF.E[i])));
  ["lowerL", "lowerR"].forEach((n, i) => pivotRot(parts[n].g, REF.E[i], E[i], ang(E[i], J[i]) - ang(REF.E[i], REF.J[i])));
  parts.plate.g.matrix.makeTranslation(F[0] - REF.F[0], 0, F[1] - REF.F[1]);
  parts.tri.g.matrix.makeTranslation(E[1][0] - REF.E[1][0], 0, E[1][1] - REF.E[1][1]);
  const B = [ROBOT.b + ROBOT.o1[0], ROBOT.o1[1]];
  pivotRot(parts.rod1.g, B, B, ang(A[1], E[1]) - ang(REF.A[1], REF.E[1]));
  const lowRef = TRI.low, lowNew = [E[1][0] + ROBOT.o2[0], E[1][1] + ROBOT.o2[1]];
  pivotRot(parts.rod2.g, lowRef, lowNew, ang(E[1], J[1]) - ang(REF.E[1], REF.J[1]));
  for (const n of PART_NAMES) parts[n].g.matrixWorldNeedsUpdate = true;
  // hose
  const ej = [F[0] + 0.07, F[1] - 0.035];
  updateTube(hose, [
    V(ej[0] + 0.02, -0.02, ej[1]), V(J[0][0] + 0.01, -0.06, J[0][1] + 0.03),
    V(lerp(J[0][0], E[0][0], 0.5), -0.07, lerp(J[0][1], E[0][1], 0.5)), V(E[0][0] + 0.03, -0.075, E[0][1] - 0.05),
    V(-0.24, -0.13, 0.02), V(-0.205, -0.14, 0.10),
  ]);
  return pose;
}

// ---- vendor CAD (local server only): _cad/duopod.glb, git-ignored. ?cad=0 keeps the procedural model.
const CAD_PART = {
  "MT_WST00065198:1": "head", "MT_WST00055121:1": "head", "MT_WST00055121:2": "head",
  "MT_WST00061011:1": "upperL", "MT_WST00061012:1": "upperR", "MT_WST00061013:1": "lowerL", "MT_WST00061013:2": "lowerR",
  "MT_WST00065199:1": "plate", "MT_WST00060925:1": "tri",
  "MT_WST00061015:1": "rod1", "MT_WST00061015:2": "rod1", "MT_WST00061014:1": "rod2", "MT_WST00061014:2": "rod2",
};
let cadLoaded = false;
function cadMaterial(src, part) {
  const c = src.color || new THREE.Color(0.6, 0.6, 0.6), l = (c.r + c.g + c.b) / 3;
  if (part === "head") return l > 0.7 ? M.aluClear : M.headAlu;   // vendor logo faces on the head become plain aluminium
  if (c.r > 0.3 && c.g < 0.1) return M.plateRed;
  if (l < 0.06) return M.carbonGloss;
  if (c.r > c.g * 1.6 && l < 0.2) return M.headAlu;          // brownish label faces become plain aluminium
  if (l > 0.7) return M.aluClear;
  return M.headAlu;
}
function cadCheck() {     // mm between the CAD plate's bottom face centre and the logic flange point
  if (!cadLoaded || !robotPose) return null;
  const box = new THREE.Box3().setFromObject(parts.plate.g.children.find((o) => o.userData.cad));
  const c = box.getCenter(V(0, 0, 0)); world.worldToLocal(c);
  const bottom = world.worldToLocal(V(0, box.min.y, 0)).z;     // three y-up: box.min.y is the lowest point
  return Math.hypot(c.x - robotPose.flange[0], bottom - robotPose.flange[1]) * 1000;
}
(async () => {
  // the vendor GLB is never published: try it only on a local server
  if (new URLSearchParams(location.search).get("cad") === "0" || !["localhost", "127.0.0.1"].includes(location.hostname)) return;
  try {
    const { GLTFLoader } = await import("three/addons/loaders/GLTFLoader.js");
    const gltf = await new GLTFLoader().loadAsync("_cad/duopod.glb");
    const root = gltf.scene; root.updateMatrixWorld(true);
    const C = new THREE.Matrix4().set(0, 1, 0, 0, -1, 0, 0, 0.065, 0, 0, 1, 0, 0, 0, 0, 1);
    const found = new Set(), byName = {};
    for (const k of Object.keys(CAD_PART)) byName[THREE.PropertyBinding.sanitizeNodeName(k)] = k;   // GLTFLoader turns ":" into "_"
    root.traverse((o) => {
      const key = byName[o.name], part = key && CAD_PART[key]; if (!part || o.userData.taken) return;
      found.add(key); o.traverse((c) => { c.userData.taken = true; });
      const holder = new THREE.Group(); holder.userData.cad = true; holder.matrixAutoUpdate = false;
      holder.matrix.copy(C).multiply(o.matrixWorld);
      const clone = o.clone(false); clone.position.set(0, 0, 0); clone.quaternion.identity(); clone.scale.set(1, 1, 1);
      if (o.isMesh) { clone.geometry = o.geometry; }
      o.children.filter((c) => c.isMesh).forEach((c) => { const cc = c.clone(); clone.add(cc); });
      clone.traverse((m) => { if (m.isMesh) { m.material = cadMaterial(m.material, part); m.castShadow = m.receiveShadow = true; } });
      holder.add(clone); parts[part].g.add(holder);
    });
    if (found.size < Object.keys(CAD_PART).length) throw new Error("vendor GLB: missing nodes " + Object.keys(CAD_PART).filter((n) => !found.has(n)).join(", "));
    for (const n of PART_NAMES) parts[n].proc.visible = false;
    cadLoaded = true;
    window.dispatchEvent(new Event("cadloaded"));
  } catch (err) {
    console.info("vendor CAD not loaded, procedural DuoPod in use:", String(err.message || err));
  }
})();

// vacuum status LED on the ejector (green while the vacuum is on)
const vacLedMat = M.lampGreen.clone();
{ const o = mesh(new THREE.SphereGeometry(0.0045, 12, 8), vacLedMat, parts.plate.g); o.position.set(0.07, -0.0165, -0.744 - 0.035); }
function setGripperFx(on) { vacLedMat.emissiveIntensity = on ? 2.2 : 0; }
