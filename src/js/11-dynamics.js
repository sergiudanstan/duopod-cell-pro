// ---------------------------------------------------------------- rigid-body dynamics (still inside the logic block: pure JS)
// Mass properties per part from the vendor STEP (tools/cad/mass.py: signed-volume integrals on the tessellation,
// aluminium 2700 / carbon 1600 kg/m^3). Robot-plane frame: x across, z up, origin on the motor-axis line, CAD pose.
// The STEP models the carbon tubes with thick walls, so the arm masses are on the high side (conservative torques).
const BODY_REF = {
  upper: { m: 7.1507, com: [0.32956, 0.05604], J: 0.16993 },   // MT_WST00061011/12, right arm values (left mirrored)
  lower: { m: 4.4467, com: [0.32795, -0.30531], J: 0.36254 },  // MT_WST00061013
  plate: { m: 2.5100, com: [-0.00148, -0.70643], J: 0.01372 }, // MT_WST00065199 (translates)
  tri: { m: 1.2555, com: [0.55761, 0.15112], J: 0.00583 },     // MT_WST00060925 (translates with the right elbow)
  rod1: { m: 2 * 0.2139, com: [0.39450, 0.17188], J: 2 * 0.0034 },   // MT_WST00061015 x2 (rotates like the right upper arm)
  rod2: { m: 2 * 0.4540, com: [0.20243, -0.25510], J: 2 * 0.03139 }, // MT_WST00061014 x2 (rotates like the right lower arm)
};
// Recommended motor for M1/M2 on the Autonox page: Beckhoff AM8053-0N21. Data: Beckhoff Infosys, AM805x technical data,
// winding N at 400 V. The Autonox gearbox ratio is not published: PARAMS.ratio is an assumption (what-if).
const MOTOR = { name: "Beckhoff AM8053-0N21", M0: 11.40, Mn: 2.70, nN: 7000, Mmax: 53.14, nMax: 9000, J: 5.93e-4 };
// Vacuum: foam pad on corrugated board. 30 kPa effective (porous board, conservative; a sealed part would see ~60 kPa),
// 70 % of the pad area sealed, friction 0.5 for the lateral load, safety factor 2 required.
const VAC = { dp: 30e3, sealed: 0.70, mu: 0.5, sfReq: 2.0 };

const rot2 = (v, a) => [v[0] * Math.cos(a) - v[1] * Math.sin(a), v[0] * Math.sin(a) + v[1] * Math.cos(a)];
const angOf = (a, b) => Math.atan2(b[1] - a[1], b[0] - a[0]);
const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));   // angle differences near ±pi (left arm) must not jump by 2pi
const REFP = (() => {   // reference joint points, world plane coordinates (z includes the motor-axis height)
  const zm = ROBOT.zm, E = ROBOT.ref.elbow;
  return {
    A: [[-ROBOT.b, zm], [ROBOT.b, zm]], E: [[-E[0], zm + E[1]], [E[0], zm + E[1]]],
    J: [[-ROBOT.e, zm - 0.705], [ROBOT.e, zm - 0.705]], F: [0, zm - 0.744],
  };
})();
const W = (c) => [c[0], ROBOT.zm + c[1]];                 // CAD-pose plane coordinate -> world plane
const mirror = (c) => [-c[0], c[1]];

// Body states (mass, centre of mass, angle from the CAD pose, planar inertia) for a TCP.
function bodyStates(tcp, holding) {
  const pose = armPose(tcp); if (!pose) return null;
  const A = pose.motors, E = pose.elbows, Jt = pose.joints, F = pose.flange, out = [];
  const moved = (ref, pivRef, pivNew, d) => { const r = rot2([ref[0] - pivRef[0], ref[1] - pivRef[1]], d); return [pivNew[0] + r[0], pivNew[1] + r[1]]; };
  for (let i = 0; i < 2; i++) {
    const side = (c) => W(i === 0 ? mirror(c) : c);
    const dU = wrapPi(angOf(A[i], E[i]) - angOf(REFP.A[i], REFP.E[i]));
    out.push({ m: BODY_REF.upper.m, J: BODY_REF.upper.J, p: moved(side(BODY_REF.upper.com), REFP.A[i], A[i], dU), phi: dU });
    const dL = wrapPi(angOf(E[i], Jt[i]) - angOf(REFP.E[i], REFP.J[i]));
    out.push({ m: BODY_REF.lower.m, J: BODY_REF.lower.J, p: moved(side(BODY_REF.lower.com), REFP.E[i], E[i], dL), phi: dL });
  }
  const dF = [F[0] - REFP.F[0], F[1] - REFP.F[1]];
  out.push({ m: BODY_REF.plate.m, J: 0, p: [W(BODY_REF.plate.com)[0] + dF[0], W(BODY_REF.plate.com)[1] + dF[1]], phi: 0 });
  out.push({ m: TOOL.mass, J: 0, p: [F[0], F[1] - TOOL.len / 2], phi: 0 });
  if (holding) out.push({ m: PARAMS.payload, J: 0, p: [tcp[0], tcp[1] - CARTON.h / 2], phi: 0 });
  const dE = [E[1][0] - REFP.E[1][0], E[1][1] - REFP.E[1][1]];
  out.push({ m: BODY_REF.tri.m, J: 0, p: [W(BODY_REF.tri.com)[0] + dE[0], W(BODY_REF.tri.com)[1] + dE[1]], phi: 0 });
  const B = [REFP.A[1][0] + ROBOT.o1[0], REFP.A[1][1] + ROBOT.o1[1]];
  const dU1 = wrapPi(angOf(A[1], E[1]) - angOf(REFP.A[1], REFP.E[1]));
  out.push({ m: BODY_REF.rod1.m, J: BODY_REF.rod1.J, p: moved(W(BODY_REF.rod1.com), B, B, dU1), phi: dU1 });
  const lowRef = [REFP.E[1][0] + ROBOT.o2[0], REFP.E[1][1] + ROBOT.o2[1]], lowNew = [E[1][0] + ROBOT.o2[0], E[1][1] + ROBOT.o2[1]];
  const dL1 = wrapPi(angOf(E[1], Jt[1]) - angOf(REFP.E[1], REFP.J[1]));
  out.push({ m: BODY_REF.rod2.m, J: BODY_REF.rod2.J, p: moved(W(BODY_REF.rod2.com), lowRef, lowNew, dL1), phi: dL1 });
  return out;
}
// Potential energy (for the static virtual-work test).
function potential(th, holding) { const b = bodyStates(fk2(th), holding); return b.reduce((s, x) => s + x.m * G * x.p[1], 0); }
// Generalised (arm-side) joint torques by d'Alembert / virtual work:
//   tau_j = sum_b m_b (a_b + g z) . dp_b/dtheta_j + J_b phi''_b dphi_b/dtheta_j
function jointDynamics(t, holding) {
  const h = 2e-3, d = 1e-6;
  const pts = [t - h, t, t + h].map((x) => robotTcp(clamp(x, 0, DURATION)));
  const st = pts.map((p) => bodyStates(p, holding)), th = pts.map((p) => ik2(p));
  const thd = [0, 1].map((j) => (th[2][j] - th[0][j]) / (2 * h));
  const thdd = [0, 1].map((j) => (th[2][j] - 2 * th[1][j] + th[0][j]) / (h * h));
  const th0 = th[1], tau = [0, 0];
  for (let j = 0; j < 2; j++) {
    const a = th0.slice(), b = th0.slice(); a[j] += d; b[j] -= d;
    const sa = bodyStates(fk2(a), holding), sb = bodyStates(fk2(b), holding);
    st[1].forEach((body, k) => {
      const acc = [(st[2][k].p[0] - 2 * body.p[0] + st[0][k].p[0]) / (h * h), (st[2][k].p[1] - 2 * body.p[1] + st[0][k].p[1]) / (h * h)];
      const phidd = (st[2][k].phi - 2 * body.phi + st[0][k].phi) / (h * h);
      const dp = [(sa[k].p[0] - sb[k].p[0]) / (2 * d), (sa[k].p[1] - sb[k].p[1]) / (2 * d)], dphi = (sa[k].phi - sb[k].phi) / (2 * d);
      tau[j] += body.m * (acc[0] * dp[0] + (acc[1] + G) * dp[1]) + body.J * phidd * dphi;
    });
  }
  return { tau, thd, thdd };
}
// Static arm torques for a pose (a = 0): used by the virtual-work test.
function staticTorque(tcp, holding) {
  const th = ik2(tcp), d = 1e-6, tau = [0, 0];
  for (let j = 0; j < 2; j++) {
    const a = th.slice(), b = th.slice(); a[j] += d; b[j] -= d;
    const sa = bodyStates(fk2(a), holding), sb = bodyStates(fk2(b), holding);
    sa.forEach((x, k) => { tau[j] += x.m * G * (sa[k].p[1] - sb[k].p[1]) / (2 * d); });
  }
  return tau;
}

// ---------------------------------------------------------------- motor side, vacuum, verdicts
// S1 line simplified between the published points (0, M0) and (nN, Mn) at 400 V.
const contTorqueAt = (n) => Math.max(MOTOR.Mn, MOTOR.M0 - (MOTOR.M0 - MOTOR.Mn) * Math.min(1, Math.abs(n) / MOTOR.nN));
function motorStats() {
  const i = PARAMS.ratio, eta = PARAMS.eta, first = segs.find((s) => s.kind === "down"), last = cartons.reduce((m, c) => Math.max(m, c.place || 0), 0);
  const t0 = first ? first.t0 : 0, t1 = last || DURATION;
  const tm = [], nm = [];
  let peak = [0, 0], nPeak = [0, 0], sq = [0, 0], nAbs = [0, 0], cnt = 0;
  for (let k = 0; k < S.n; k++) {
    const d = S.dyn[k]; if (!d) { tm.push([NaN, NaN]); nm.push([NaN, NaN]); continue; }
    const tq = [0, 1].map((j) => {
      const load = d.tau[j] / i, motoring = d.tau[j] * d.thd[j] >= 0;
      // measured physics torque already contains the reflected rotor inertia (MuJoCo armature)
      return (motoring ? load / eta : load * eta) + (d.measured ? 0 : MOTOR.J * i * d.thdd[j]);
    });
    const n = [0, 1].map((j) => d.thd[j] * i * 60 / (2 * Math.PI));
    tm.push(tq); nm.push(n);
    const t = k * S.dt;
    for (let j = 0; j < 2; j++) { peak[j] = Math.max(peak[j], Math.abs(tq[j])); nPeak[j] = Math.max(nPeak[j], Math.abs(n[j])); }
    if (t >= t0 && t <= t1) { cnt++; for (let j = 0; j < 2; j++) { sq[j] += tq[j] * tq[j]; nAbs[j] += Math.abs(n[j]); } }
  }
  const rms = sq.map((v) => Math.sqrt(v / Math.max(1, cnt))), nMean = nAbs.map((v) => v / Math.max(1, cnt));
  const cont = nMean.map(contTorqueAt);
  const armPeak = [0, 1].map((j) => Math.max(...S.dyn.filter(Boolean).map((d) => Math.abs(d.tau[j]))));
  const ok = { peak: Math.max(...peak) <= PARAMS.torqueLimit * MOTOR.Mmax, rms: rms.every((r, j) => r <= cont[j]), speed: Math.max(...nPeak) <= 0.95 * MOTOR.nMax };
  return { tm, nm, peak, rms, nPeak, nMean, cont, armPeak, ok, window: [t0, t1] };
}
function vacuumStats() {
  const area = TOOL.padX * TOOL.padY * VAC.sealed, avail = VAC.dp * area;
  let worst = Infinity, at = 0, req = 0;
  for (let k = 0; k < S.n; k++) {
    const t = k * S.dt;
    if (!cartons.some((c) => c.pick != null && t >= c.pick && t < c.bAt)) continue;
    const a = S.acc3[k], m = PARAMS.payload;
    const need = m * Math.max(0, G + a[1]) + m * Math.abs(a[0]) / VAC.mu;
    const sf = avail / Math.max(1e-9, need);
    if (sf < worst) { worst = sf; at = t; req = need; }
  }
  return { avail, area, sf: worst, at, need: req, ok: worst >= VAC.sfReq };
}
function minClearance() { return S.clear.reduce((b, c, k) => (c.m < b.m ? { ...c, at: k * S.dt } : b), { m: Infinity }); }
// Every limit of the cell for the current build; names the first one that fails.
function checkLimits() {
  const fails = [];
  if (!S.th.every(Boolean)) fails.push("reach");
  // queued cartons touch; MuJoCo soft contacts compress up to a few tenths of a millimetre, so physics data gets 1 mm
  if (Math.max(...S.overlap) > (PHYS ? 1e-3 : 1e-6)) fails.push("carton overlap");
  if (minClearance().m < 0.010) fails.push("clearance < 10 mm");
  if (Math.max(...S.gripPen) > 1e-6) fails.push("gripper hits a carton");
  if (violations.length) fails.push("descent without carton");
  if (!S.motor.ok.peak) fails.push(`motor peak > ${(PARAMS.torqueLimit * 100).toFixed(0)} % Mmax`);
  if (!S.motor.ok.rms) fails.push("motor RMS > S1");
  if (!S.motor.ok.speed) fails.push("motor speed > 95 % nmax");
  if (!S.vac.ok) fails.push(`vacuum safety < ${VAC.sfReq}`);
  if (cartons.some((c) => c.place == null)) fails.push("not all cartons placed");
  return fails;
}
// Time-optimal scaling: smallest PARAMS.scale (fastest moves) for which every limit holds. Bisection on [lo, hi].
function optimizeScale(lo = PARAM_RANGE.scale[0], hi = PARAM_RANGE.scale[1], iters = 9) {
  const keep = PARAMS.scale, test = (k) => { PARAMS.scale = k; rebuild(); return checkLimits(); };
  if (test(hi).length) { const f = checkLimits(); PARAMS.scale = keep; rebuild(); return { scale: null, binding: f[0], fails: f }; }
  let binding = test(lo)[0] || null;
  if (!binding) return { scale: lo, binding: "slider lower bound" };
  for (let n = 0; n < iters; n++) {
    const mid = (lo + hi) / 2, f = test(mid);
    if (f.length) { lo = mid; binding = f[0]; } else hi = mid;
  }
  PARAMS.scale = Math.ceil(hi * 1000) / 1000; rebuild();
  return { scale: PARAMS.scale, binding };
}
rebuild();
// @@logic-end
