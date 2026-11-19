// Checks the cell logic in index.html without a browser.
// Usage: node tests/run.mjs [file.html] [--perturb]
//   --perturb lets the escapement release on vacuum OK (robot still low): the overlap and clearance checks must fail.
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const file = args.find((a) => !a.startsWith("--")) || join(here, "..", "index.html");
const perturb = args.includes("--perturb");
const html = readFileSync(file, "utf8");
const m = html.match(/\/\/ @@logic-start[^\n]*\n([\s\S]*?)\/\/ @@logic-end/);
if (!m) { console.error("logic block not found"); process.exit(2); }
let code = m[1];
if (perturb) {
  const before = code;
  code = code.replace("const ESC_ON_GRIP = false;", "const ESC_ON_GRIP = true;");
  if (code === before) { console.error("perturbation target not found"); process.exit(2); }
}
const L = new Function(code + `
return { ROBOT, TOOL, CARTON, BELTS, GUIDE, Z, P, N_CARTONS, ik2, fk2, armPose, jacobian, cond2, transmission, cycleStats, cartonAt, DEG, STOP_Y,
  PARAMS, PARAMS0, PARAM_RANGE, MOTOR, VAC, rebuild, checkLimits, minClearance, optimizeScale, staticTorque, potential,
  get S() { return S; }, get segs() { return segs; }, get cartons() { return cartons; }, get DURATION() { return DURATION; },
  get escape() { return escape; }, get violations() { return violations; }, get MOVES() { return MOVES; } };`)();
const { ROBOT, TOOL, CARTON, BELTS, DEG } = L;
let S = L.S;

let fails = 0;
const check = (ok, msg) => { console.log(`${ok ? "PASS" : "FAIL"}  ${msg}`); if (!ok) fails++; };
const mm = (v) => (v * 1000).toFixed(1);
const flangeTop = ROBOT.mountZ - ROBOT.datasheet.below;           // top line of the TCP working range (flange plane)

// 1. kinematics against the vendor CAD pose and IK o FK on a grid
{
  const tcp = [0, ROBOT.zm - 0.744 - TOOL.len], p = L.armPose(tcp);
  const errE = Math.max(...p.elbows.map((E, i) => Math.hypot(Math.abs(E[0]) - ROBOT.ref.elbow[0], E[1] - ROBOT.zm - ROBOT.ref.elbow[1])));
  check(errE < 1e-4, `CAD pose: elbows within ${mm(errE)} mm of the STEP bore centres (theta ${(p.th[0] * DEG).toFixed(2)} deg)`);
  let worst = 0;
  for (let i = 0; i <= 30; i++) for (let j = 0; j <= 20; j++) {
    const t = [-0.3 + 0.6 * i / 30, flangeTop - 0.4 * j / 20 - TOOL.len];
    const q = L.ik2(t), f = q && L.fk2(q);
    worst = Math.max(worst, f ? Math.hypot(f[0] - t[0], f[1] - t[1]) : Infinity);
  }
  check(worst < 1e-6, `IK o FK identity on 600 x 400 grid: worst ${(worst * 1e6).toFixed(3)} um`);
}
// 2. working range: datasheet 1000 x 400 and the required 600 x 400
for (const [w, h, label] of [[ROBOT.datasheet.width, ROBOT.datasheet.height, "datasheet 1000 x 400"], [0.6, 0.4, "required 600 x 400"]]) {
  let miss = 0, muMin = 180, muMax = 0, cMax = 0;
  for (let i = 0; i <= 40; i++) for (let j = 0; j <= 16; j++) {
    const p = L.armPose([-w / 2 + w * i / 40, flangeTop - h * j / 16 - TOOL.len]);
    if (!p) { miss++; continue; }
    const mu = L.transmission(p); muMin = Math.min(muMin, ...mu); muMax = Math.max(muMax, ...mu);
    cMax = Math.max(cMax, L.cond2(L.jacobian(p.th)));
  }
  check(miss === 0 && muMin >= 30 && muMax <= 150 && cMax < 5,
    `${label}: reachable, transmission ${muMin.toFixed(0)}..${muMax.toFixed(0)} deg, Jacobian condition <= ${cMax.toFixed(2)}`);
}
// 3. pick and place points sit inside the required rectangle (flange plane)
{
  const inside = (p) => Math.abs(p[0]) <= 0.3 && p[1] + TOOL.len <= flangeTop + 1e-9 && p[1] + TOOL.len >= flangeTop - 0.4 - 1e-9;
  check([L.P.pick, L.P.place, L.P.ready].every(inside), "pick, place and travel points lie inside the 600 x 400 rectangle");
}
// 4. every sample of the cycle has an IK solution; the plate stays level by construction (two parallelograms)
check(S.th.every(Boolean), `IK solved at all ${S.n} samples (60 Hz)`);
{
  const muAll = S.mu.flat();
  check(Math.min(...muAll) >= 30 && Math.max(...muAll) <= 150, `transmission angle over the cycle ${Math.min(...muAll).toFixed(0)}..${Math.max(...muAll).toFixed(0)} deg`);
}
// 5. cartons: each picked once, placed once, in order, inside the guides of belt B, spacing on B
{
  const C = L.cartons;
  check(C.length === L.N_CARTONS && C.every((c) => c.pick != null && c.place != null && c.place > c.pick), `${C.length} cartons each picked once and placed once`);
  check(C.every((c, k) => k === 0 || c.pick > C[k - 1].place), "cartons are handled in arrival order, one at a time");
  const st = L.cycleStats();
  check(st.pitchB - CARTON.y >= 0.10, `belt B pitch ${mm(st.pitchB)} mm: gap ${mm(st.pitchB - CARTON.y)} mm between cartons`);
  const p = L.cartonAt(C[3], C[3].place + 0.01);
  const side = BELTS.B.w / 2 - CARTON.x / 2 - Math.abs(p[0] - BELTS.B.x);
  check(side >= 0.05 && p[2] === 0, `placed carton sits on belt B with ${mm(side)} mm to each guide`);
  check(st.perMin >= 25, `cadence ${st.perMin.toFixed(1)} cartons/min (period ${st.period.toFixed(3)} s), target >= 25`);
}
// 6. collisions: carton-carton overlap, clearances robot/load vs guides, stopper, cartons
{
  const ov = Math.max(...S.overlap);
  check(ov <= 1e-6, `no carton overlaps another (max depth ${mm(ov)} mm)`);
  let worst = { m: Infinity }, at = 0;
  S.clear.forEach((c, i) => { if (c.m < worst.m) { worst = c; at = i * S.dt; } });
  check(worst.m >= 0.010, `min clearance ${mm(worst.m)} mm (${worst.what} at ${at.toFixed(2)} s), required >= 10 mm`);
  const pen = Math.max(...S.gripPen);
  check(pen <= 1e-6, `gripper never penetrates a carton (max ${mm(pen)} mm)`);
}
// 7. dynamics: virtual work, motor sizing, vacuum
{
  let worst = 0;
  for (const tcp of [L.P.pick, L.P.place, [0, 0.25], [0.3, 0.30], [-0.3, 0.0]]) for (const holding of [false, true]) {
    const th = L.ik2(tcp), st = L.staticTorque(tcp, holding), d = 1e-5;
    for (let j = 0; j < 2; j++) {
      const a = th.slice(), b = th.slice(); a[j] += d; b[j] -= d;
      worst = Math.max(worst, Math.abs(st[j] - (L.potential(a, holding) - L.potential(b, holding)) / (2 * d)));
    }
  }
  check(worst < 1e-3, `static torques equal dV/dtheta (virtual work) at 10 poses: worst ${worst.toExponential(2)} Nm`);
  const m = S.motor;
  check(S.dyn.every((d) => !d || d.tau.every(Number.isFinite)) && m.peak.every(Number.isFinite), "arm and motor torques finite at every sample");
  console.log(`INFO  arm-side peak ${m.armPeak.map((v) => v.toFixed(0)).join(" / ")} Nm; ratio i = ${L.PARAMS.ratio} (assumption), eta ${L.PARAMS.eta}`);
  check(m.ok.peak, `motor peak ${m.peak.map((v) => v.toFixed(2)).join(" / ")} Nm <= ${(L.PARAMS.torqueLimit * 100).toFixed(0)} % of Mmax ${L.MOTOR.Mmax} Nm (${L.MOTOR.name})`);
  check(m.ok.rms, `motor RMS ${m.rms.map((v) => v.toFixed(2)).join(" / ")} Nm <= S1 ${m.cont.map((v) => v.toFixed(1)).join(" / ")} Nm at mean speed`);
  check(m.ok.speed, `motor speed ${m.nPeak.map((v) => v.toFixed(0)).join(" / ")} rpm <= 95 % of ${L.MOTOR.nMax} rpm`);
  check(S.vac.ok, `vacuum safety factor ${S.vac.sf.toFixed(2)} >= ${L.VAC.sfReq} (available ${S.vac.avail.toFixed(0)} N, worst need ${S.vac.need.toFixed(0)} N at ${S.vac.at.toFixed(2)} s)`);
  check(L.violations.length === 0, `smart cycle: every direct descent starts with the photo-eye confirmed (${L.cycleStats().smart} direct returns)`);
  check(L.checkLimits().length === 0, `all limits hold at the default parameters`);
}
// 8. smart cycle beats the simple cycle with the same limits
{
  const smart = L.cycleStats().perMin;
  L.PARAMS.smart = false; L.rebuild();
  const simple = L.cycleStats().perMin, failsSimple = L.checkLimits();
  L.PARAMS.smart = true; L.rebuild();
  check(smart > simple && failsSimple.length === 0, `smart cycle ${smart.toFixed(1)}/min > simple cycle ${simple.toFixed(1)}/min, both within all limits`);
}
// 9. time-optimal scaling respects every limit and names the binding one
{
  const o = L.optimizeScale();
  const fails = L.checkLimits(), st = L.cycleStats();
  check(o.scale != null && fails.length === 0 && typeof o.binding === "string",
    `optimizer: scale ${o.scale} -> ${st.perMin.toFixed(1)}/min, all limits hold, binding limit: ${o.binding}`);
  Object.assign(L.PARAMS, L.PARAMS0); L.rebuild();
}
// 10. what-if: every parameter at both ends of its range gives a valid cycle or named failures, never NaN
{
  let bad = [];
  const notes = [];
  for (const [k, r] of Object.entries(L.PARAM_RANGE)) for (const v of [r[0], r[1]]) {
    Object.assign(L.PARAMS, L.PARAMS0, { [k]: v });
    try {
      L.rebuild();
      const f = L.checkLimits(), st = L.cycleStats();
      const finite = Number.isFinite(st.perMin) && L.S.motor.peak.every(Number.isFinite) && Number.isFinite(L.S.vac.sf);
      if (!finite) bad.push(`${k}=${v}: NaN`);
      notes.push(`${k}=${v}: ${st.perMin.toFixed(1)}/min${f.length ? " [" + f.join(", ") + "]" : ""}`);
    } catch (e) { bad.push(`${k}=${v}: ${e.message}`); }
  }
  Object.assign(L.PARAMS, L.PARAMS0); L.rebuild();
  for (const n of notes) console.log("INFO  what-if " + n);
  check(bad.length === 0, `what-if sweep: ${notes.length} edge cases, no NaN or exception${bad.length ? " -> " + bad.join("; ") : ""}`);
}

console.log(`\n${fails ? "FAILED" : "OK"}: ${fails} failing check(s)${perturb ? " (perturbed run)" : ""}`);
process.exit(fails ? 1 : 0);
