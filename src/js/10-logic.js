// @@logic-start  (pure JS, no DOM and no three.js: tests/run.mjs evaluates this block in Node)
// World frame: metres, z up, belt top z = 0. The robot works in the plane y = 0 (x across the belts, z up).
const DEG = 180 / Math.PI, RAD = Math.PI / 180;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const lerp = (a, b, s) => a + (b - a) * s;
const ease = (s) => s * s * s * (10 + s * (6 * s - 15));          // minimum-jerk position profile

// ---------------------------------------------------------------- robot: Autonox DuoPod RVS2-1000-3kg (A_00913)
// Joint centres fitted on the bearing bores of the vendor STEP (3D-model_A_00913.stp), CAD pose, metres:
//   motor axes (±0.125, 0); elbows (±0.56495, 0.09458); plate joints (±0.090, -0.705); flange (TCP) (0, -0.744)
//   => L1 = 0.4500, L2 = 0.9300. Mounting face 0.170 above the motor axes (TCP 914 mm below it, connecting dims p.1).
// Orientation parallelograms (right arm only): rod pair 1 head -> triangle, rod pair 2 triangle -> plate.
const ROBOT = {
  b: 0.125, L1: 0.450, L2: 0.930, e: 0.090, hJ: 0.039, mountAbove: 0.170,
  o1: [0.04153, 0.11641],          // triangle top pivot - elbow (= head rod pivot - motor axis)
  o2: [-0.11434, 0.05052],         // triangle lower pivot - elbow (= plate rod pivot - plate joint)
  ref: { tcp: [0, -0.744], elbow: [0.56495, 0.09458] },
  mountZ: 1.330,                   // mounting face above the belt top
  datasheet: { width: 1.000, height: 0.400, below: 0.914, arcDepth: 0.510 },
};
ROBOT.zm = ROBOT.mountZ - ROBOT.mountAbove;             // motor axis height
const TOOL = { len: 0.110, padX: 0.160, padY: 0.220, mass: 1.2 };   // vacuum gripper: flange -> foam face; 1.2 kg estimate (aluminium + ejector)
const G = 9.81;

// ---------------------------------------------------------------- what-if parameters (UI sliders; tests sweep the ranges)
const PARAMS = { vA: 0.40, vB: 0.40, payload: 3.0, travel: 0.280, ratio: 40, eta: 0.90, scale: 1.0, smart: true, torqueLimit: 0.85 };
const PARAM_RANGE = {
  vA: [0.20, 0.60, 0.05, "m/s"], vB: [0.20, 0.60, 0.05, "m/s"], payload: [1.0, 3.0, 0.5, "kg"],
  travel: [0.200, 0.300, 0.010, "m"], ratio: [20, 80, 5, ""], scale: [0.50, 1.50, 0.05, ""], torqueLimit: [0.50, 1.00, 0.05, ""],
};
const PARAMS0 = { ...PARAMS };

// ---------------------------------------------------------------- cell
const CARTON = { x: 0.180, y: 0.250, h: 0.120 };          // across the belt, along the belt, height
const BELTS = {
  A: { x: 0.20, w: 0.22, v: PARAMS.vA, y0: -1.75, y1: 0.40 },    // infeed (operator's left, seen from +y); guides 220 mm apart keep the carton
  //   centred under the gripper (the MuJoCo physics run showed a 300 mm gap lets real cartons drift sideways)
  B: { x: -0.20, w: 0.30, v: PARAMS.vB, y0: -0.55, y1: 1.75 },   // outfeed, runs continuously
};
const GUIDE = { t: 0.010, top: 0.060 };
const STOP_Y = CARTON.y / 2;                              // carton front edge at the stopper -> carton centred on y = 0
const BLADE_Y = STOP_Y - CARTON.y - 0.033;                // escapement blade: queue head front edge stops here
const SPAWN_Y = BELTS.A.y0 + CARTON.y + 0.05;             // front edge of a new carton entering the cell
const T_IN = 1.25;                                        // infeed: one carton every 1.25 s from upstream
const N_CARTONS = 12;

const Z = {
  pick: CARTON.h,            // TCP (foam face) on the carton top
  place: CARTON.h,           // carton bottom on belt B
  travel: PARAMS.travel,     // TCP height for transfers (default 280: carton bottom 160 mm, 100 mm over the guides)
  clear: 2 * CARTON.h + 0.020,   // escapement may release once the held carton is this high (TCP)
};
const P = { pick: [BELTS.A.x, Z.pick], place: [BELTS.B.x, Z.place], ready: [BELTS.A.x, Z.travel] };
const TIMES = { grip: 0.12, release: 0.10, settle: 0.05 };   // move times live in MOVES
const PLACE_GAP = 0.030;                                  // belt B: previous carton at least 30 mm clear of the drop zone

// ---------------------------------------------------------------- kinematics in the plane (x, z), world frame
const ARMS = [-1, 1];   // s = -1 left arm (x < 0), +1 right arm
function plateJoint(tcp, s) { return [tcp[0] + s * ROBOT.e, tcp[1] + TOOL.len + ROBOT.hJ]; }
function motorAxis(s) { return [s * ROBOT.b, ROBOT.zm]; }
// Elbow for one arm, elbow-out branch; null if out of reach.
function elbowFor(tcp, s) {
  const A = motorAxis(s), Pj = plateJoint(tcp, s);
  const dx = Pj[0] - A[0], dz = Pj[1] - A[1], d = Math.hypot(dx, dz);
  const { L1, L2 } = ROBOT;
  if (d > L1 + L2 || d < Math.abs(L1 - L2) || d < 1e-9) return null;
  const a = (L1 * L1 - L2 * L2 + d * d) / (2 * d), h = Math.sqrt(Math.max(0, L1 * L1 - a * a));
  const mx = A[0] + a * dx / d, mz = A[1] + a * dz / d;
  const e1 = [mx + h * dz / d, mz - h * dx / d], e2 = [mx - h * dz / d, mz + h * dx / d];
  return s * e1[0] > s * e2[0] ? e1 : e2;
}
// theta: upper-arm angle above the outward horizontal (CAD pose 12.13 deg). Returns [thL, thR] in rad or null.
function ik2(tcp) {
  const th = [];
  for (const s of ARMS) {
    const E = elbowFor(tcp, s); if (!E) return null;
    const A = motorAxis(s);
    th.push(Math.atan2(E[1] - A[1], s * (E[0] - A[0])));
  }
  return th;
}
function elbowsFromTheta(th) {
  return ARMS.map((s, i) => { const A = motorAxis(s); return [A[0] + s * ROBOT.L1 * Math.cos(th[i]), A[1] + ROBOT.L1 * Math.sin(th[i])]; });
}
// Forward kinematics: two circles of radius L2 around the elbows shifted by -/+e; lower branch.
function fk2(th) {
  const [EL, ER] = elbowsFromTheta(th);
  const c1 = [EL[0] + ROBOT.e, EL[1]], c2 = [ER[0] - ROBOT.e, ER[1]];   // centres seen from the plate centre
  const dx = c2[0] - c1[0], dz = c2[1] - c1[1], d = Math.hypot(dx, dz);
  if (d > 2 * ROBOT.L2 || d < 1e-9) return null;
  const h = Math.sqrt(ROBOT.L2 * ROBOT.L2 - d * d / 4), mx = (c1[0] + c2[0]) / 2, mz = (c1[1] + c2[1]) / 2;
  const p1 = [mx + h * dz / d, mz - h * dx / d], p2 = [mx - h * dz / d, mz + h * dx / d];
  const pc = p1[1] < p2[1] ? p1 : p2;                    // plate centre at joint height
  return [pc[0], pc[1] - ROBOT.hJ - TOOL.len];
}
// Full pose for drawing and checks.
function armPose(tcp) {
  const th = ik2(tcp); if (!th) return null;
  const elbows = elbowsFromTheta(th);
  const joints = ARMS.map((s) => plateJoint(tcp, s));
  const flange = [tcp[0], tcp[1] + TOOL.len];
  return { tcp, th, elbows, joints, flange, motors: ARMS.map(motorAxis) };
}
// dx/dtheta (2x2) by central differences of the forward kinematics.
function jacobian(th) {
  const h = 1e-6, J = [[0, 0], [0, 0]];
  for (let j = 0; j < 2; j++) {
    const a = th.slice(), b = th.slice(); a[j] += h; b[j] -= h;
    const pa = fk2(a), pb = fk2(b);
    J[0][j] = (pa[0] - pb[0]) / (2 * h); J[1][j] = (pa[1] - pb[1]) / (2 * h);
  }
  return J;
}
function cond2(J) {    // ratio of singular values of a 2x2
  const a = J[0][0], b = J[0][1], c = J[1][0], d = J[1][1];
  const s1 = a * a + b * b + c * c + d * d, det = a * d - b * c;
  const r = Math.sqrt(Math.max(0, s1 * s1 - 4 * det * det));
  const sMax = Math.sqrt((s1 + r) / 2), sMin = Math.sqrt(Math.max(0, (s1 - r) / 2));
  return sMin > 0 ? sMax / sMin : Infinity;
}
// Transmission angle at each elbow (between upper and lower arm), degrees.
function transmission(pose) {
  return ARMS.map((s, i) => {
    const A = pose.motors[i], E = pose.elbows[i], Pj = pose.joints[i];
    const u = [E[0] - A[0], E[1] - A[1]], w = [Pj[0] - E[0], Pj[1] - E[1]];
    const c = (u[0] * w[0] + u[1] * w[1]) / (Math.hypot(...u) * Math.hypot(...w));
    return Math.acos(clamp(c, -1, 1)) * DEG;
  });
}

// ---------------------------------------------------------------- moves: overlapped single-axis minimum-jerk pieces (classic "goal-post" pick cycle)
// x and z each run through [t0, t1, from, to] pieces; between pieces an axis holds its value.
function makeMove(T, x0, z0, xs, zs) {
  const axis = (v0, pcs) => (t) => {
    let v = v0;
    for (const [a, b, from, to] of pcs) { if (t <= a) return v; v = t >= b ? to : lerp(from, to, ease((t - a) / (b - a))); if (t < b) return v; }
    return v;
  };
  const X = axis(x0, xs), Zf = axis(z0, zs);
  return { T, at: (t) => [X(clamp(t, 0, T)), Zf(clamp(t, 0, T))] };
}
let MOVES = null;
// Piece timings are fractions of the move; PARAMS.scale stretches every move (the optimizer searches it).
function buildMoves() {
  const k = PARAMS.scale, T = (v) => v * k, tr = Z.travel;
  MOVES = {
    down: makeMove(T(0.24), P.ready[0], tr, [], [[0, T(0.24), tr, Z.pick]]),
    transfer: makeMove(T(0.62), P.pick[0], Z.pick,
      [[T(0.16), T(0.46), P.pick[0], P.place[0]]],
      [[0, T(0.30), Z.pick, tr], [T(0.32), T(0.62), tr, Z.place]]),
    back: makeMove(T(0.52), P.place[0], Z.place,
      [[T(0.12), T(0.52), P.place[0], P.ready[0]]],
      [[0, T(0.26), Z.place, tr]]),
    // smart cycle: place -> straight down onto the next carton, no stop above the infeed
    backPick: makeMove(T(0.68), P.place[0], Z.place,
      [[T(0.12), T(0.52), P.place[0], P.pick[0]]],
      [[0, T(0.26), Z.place, tr], [T(0.44), T(0.68), tr, Z.pick]]),
  };
  MOVES.backPick.descendAt = T(0.44);
  MOVES.transfer.zDown = T(0.32);
}
// Copy PARAMS into the cell objects that the simulation and the scene read.
function applyParams() {
  BELTS.A.v = PARAMS.vA; BELTS.B.v = PARAMS.vB; Z.travel = PARAMS.travel;
  P.ready = [BELTS.A.x, Z.travel];
  buildMoves();
}

// ---------------------------------------------------------------- event simulation: infeed queue, escapement, stopper, robot, outfeed
const SIM_DT = 1 / 600;
let segs = [], cartons = [], escape = [], DURATION = 0;
// MuJoCo physics replay (duopod-cell-mujoco/tools/export_trajectory.py): when set, the cycle, the TCP, the cartons and the vacuum
// come from the recorded physics run instead of the kinematic event simulation; every check below runs on that data.
let PHYS = null;
const physIndex = (t) => { const f = clamp(t * PHYS.fps, 0, PHYS.n - 1), i = Math.floor(f); return [i, Math.min(PHYS.n - 1, i + 1), f - i]; };
function scheduleFromPhysics() {
  segs = []; escape = [];
  const st = PHYS.state, k = PHYS.k, tt = PHYS.t;
  for (let i = 0; i < PHYS.n; i++) {
    if (i === 0 || st[i] !== st[i - 1]) {
      if (segs.length) segs[segs.length - 1].t1 = tt[i];
      segs.push({ kind: st[i], t0: tt[i], t1: tt[i], k: k[i] >= 0 && st[i] !== "wait" && st[i] !== "back" ? k[i] : undefined });
    }
  }
  segs[segs.length - 1].t1 = PHYS.duration;
  let open = false;
  for (let i = 0; i < PHYS.n; i++) {
    const o = PHYS.blade[i] > 0.02;
    if (o !== open) { escape.push(o ? { t: tt[i], ev: "release", clamp: PHYS.clamp[i] > 0.005 ? 1 : -1 } : { t: tt[i], ev: "blade up" }); open = o; }
  }
  cartons = PHYS.cartons.map((fr, kk) => {
    const first = fr.findIndex((f) => f !== 0), placed = PHYS.placed.find((p) => p[1] === kk);
    let last = -1; for (let i = fr.length - 1; i >= 0; i--) if (fr[i] !== 0) { last = i; break; }
    return { k: kk, knots: [], spawn: first >= 0 ? tt[first] : null, pick: PHYS.picks[kk] ?? null, place: placed ? placed[0] : null,
      bAt: placed ? placed[0] : null, gone: last >= 0 && last < PHYS.n - 1 ? tt[last + 1] : Infinity };
  });
  DURATION = PHYS.duration;
  segs.push({ kind: "end", t0: DURATION, t1: DURATION + 1e-6 });
}
// physics replay: arm-side torque measured in MuJoCo (actuator force), joint speed / acceleration from the recorded TCP
// at the recording rate (finer differences of the interpolated path would only measure the interpolation)
function physDynamics(t) {
  const [i] = physIndex(t), h = 1 / PHYS.fps, a = Math.max(0, i - 1), b = Math.min(PHYS.n - 1, i + 1);
  const th = [a, i, b].map((k) => ik2(PHYS.tcp[k]));
  if (th.some((v) => !v)) return { tau: [NaN, NaN], thd: [0, 0], thdd: [0, 0] };
  const thd = [0, 1].map((j) => (th[2][j] - th[0][j]) / ((b - a) * h || h));
  const thdd = [0, 1].map((j) => (th[2][j] - 2 * th[1][j] + th[0][j]) / (h * h));
  return { tau: PHYS.tau[i].slice(), thd, thdd, measured: true };
}
function physCarton(kk, t) {        // [x, y, z, qw, qx, qy, qz] of the carton centre, or null
  const [i, j, a] = physIndex(t), fr = PHYS.cartons[kk], A = fr[i], B = fr[j];
  if (!A) return null;
  if (!B) return A.slice();
  return A.map((v, n) => (n < 3 ? lerp(v, B[n], a) : (a < 0.5 ? v : B[n])));
}
const LABEL = {
  wait: "Wait for carton", down: "Descend to carton", grip: "Vacuum on", transfer: "Transfer A → B",
  release: "Vacuum off, blow-off", holdB: "Hold: belt B drop zone busy", back: "Return above infeed", backPick: "Return straight to next carton", end: "Home",
};
const CMD = {
  wait: () => "WAIT DI[1:carton at stopper]=ON",
  down: () => `MOVL P[pick] z=${(Z.pick * 1000).toFixed(0)}`,
  grip: () => "DO[1:vacuum]=ON ; WAIT DI[2:vac ok]",
  transfer: () => `MOVL P[place] via z=${(Z.travel * 1000).toFixed(0)} (z up, x across, z down overlapped)`,
  release: () => "DO[1:vacuum]=OFF ; PULSE DO[2:blow-off]",
  holdB: () => "WAIT DI[3:belt B drop zone clear]=ON",
  back: () => "MOVL P[ready]",
  backPick: () => `MOVL P[pick] via z=${(Z.travel * 1000).toFixed(0)} (photo-eye confirmed before descent)`,
  end: () => "END",
};

// perturb hook for tests: true lets the escapement release as soon as the vacuum is confirmed (robot still low)
const ESC_ON_GRIP = false;

let violations = [];
function schedule() {
  if (PHYS) return scheduleFromPhysics();
  segs = []; escape = []; violations = [];
  // carton k: front edge on belt A as knots [t, yFront]; then held by the robot; then on belt B.
  cartons = Array.from({ length: N_CARTONS }, (_, k) => ({ k, knots: [], spawn: null, pick: null, place: null, bAt: null, gone: Infinity }));
  const start = [STOP_Y, BLADE_Y, BLADE_Y - CARTON.y - 0.45, BLADE_Y - CARTON.y - 0.95];   // pre-loaded at t = 0
  const due = cartons.map((c, k) => (k < start.length ? 0 : (k - start.length + 1) * T_IN));
  const front = new Array(N_CARTONS).fill(null), moving = new Array(N_CARTONS).fill(false);
  const onA = (k) => front[k] != null && cartons[k].pick == null;
  let blade = true, bladeHold = -1, clampK = -1;
  let robot = null, held = -1, nextPick = 0, t = 0, sensorOn = -1;
  const startSeg = (kind, t0, dur, extra = {}) => { const s = { kind, t0, t1: t0 + dur, ...extra }; segs.push(s); robot = { state: kind, t0, seg: s }; return s; };
  startSeg("wait", 0, 0);
  const tcpNow = () => {
    const s = robot.seg;
    if (s.move) return MOVES[s.move].at(t - s.t0);
    return s.kind === "grip" || s.kind === "holdB" ? P.pick : s.kind === "release" ? P.place : P.ready;
  };
  // time when the next carton will be settled at the stopper (null if it has not been released yet)
  const settleTime = (k) => {
    if (k >= N_CARTONS || !onA(k) || front[k] == null) return null;
    if (front[k] >= STOP_Y - 1e-6) return sensorOn >= 0 ? sensorOn + TIMES.settle : t + TIMES.settle;
    if (front[k] <= BLADE_Y + 1e-6 && blade) return null;        // still behind the closed blade
    return t + (STOP_Y - front[k]) / BELTS.A.v + TIMES.settle;  // free run to the stop (nothing ahead on belt A)
  };
  // interlock: nothing of the robot or its load below the clear height above the pick spot
  const columnClear = () => {
    if (ESC_ON_GRIP && robot.state === "transfer") return true;
    const p = tcpNow();
    return Math.abs(p[0] - BELTS.A.x) > (CARTON.x + TOOL.padX) / 2 + 0.02 || p[1] >= Z.clear;
  };
  for (; t < 120; t += SIM_DT) {
    // ---- spawn at the upstream end when there is room
    for (let k = 0; k < N_CARTONS; k++) {
      if (front[k] != null || t < due[k]) continue;
      const y = k < start.length ? start[k] : SPAWN_Y;
      const room = cartons.every((c, j) => j === k || !onA(j) || front[j] - CARTON.y >= y - 1e-9 || front[j] < y - CARTON.y);
      if (!room) continue;
      front[k] = y; cartons[k].spawn = t; cartons[k].knots.push([t, y]);
    }
    // ---- belt A: each carton runs at belt speed until the stopper, the blade, the carton ahead or the clamp stops it
    for (let k = 0; k < N_CARTONS; k++) {
      if (!onA(k)) continue;
      let lim = STOP_Y;
      if (blade && front[k] <= BLADE_Y + 1e-9) lim = BLADE_Y;
      for (let j = 0; j < N_CARTONS; j++) if (j !== k && onA(j) && front[j] > front[k]) lim = Math.min(lim, front[j] - CARTON.y);
      // the held carton is NOT an obstacle for the belt: keeping it clear is the interlock's job
      const want = clampK === k ? front[k] : Math.min(lim, front[k] + BELTS.A.v * SIM_DT);
      const mv = want > front[k] + 1e-12;
      if (mv !== moving[k]) { cartons[k].knots.push([t, front[k]]); moving[k] = mv; }
      front[k] = Math.max(front[k], want);
    }
    // ---- escapement
    if (!blade && bladeHold >= 0 && front[bladeHold] - CARTON.y > BLADE_Y + 0.005) {
      blade = true; clampK = -1; bladeHold = -1; escape.push({ t, ev: "blade up" });
    }
    if (blade) {
      const head = cartons.findIndex((c, k) => onA(k) && Math.abs(front[k] - BLADE_Y) < 1e-6);
      const zoneFree = cartons.every((c, k) => !onA(k) || front[k] <= BLADE_Y + 1e-6);
      if (head >= 0 && zoneFree && columnClear()) {
        blade = false; bladeHold = head;
        clampK = cartons.findIndex((c, k) => k !== head && onA(k) && front[k] >= front[head] - CARTON.y - 0.01 && front[k] < front[head]);
        escape.push({ t, ev: "release", k: head, clamp: clampK });
      }
    }
    // ---- photo-eye at the stopper
    const atStop = nextPick < N_CARTONS && onA(nextPick) && front[nextPick] >= STOP_Y - 1e-6;
    if (atStop && sensorOn < 0) sensorOn = t;
    if (!atStop) sensorOn = -1;
    // ---- robot
    if (robot.state === "wait") {
      if (nextPick >= N_CARTONS) { robot.seg.t1 = t; break; }
      if (sensorOn >= 0 && t - sensorOn >= TIMES.settle - 1e-9) { robot.seg.t1 = t; startSeg("down", t, MOVES.down.T, { move: "down", k: nextPick }); }
    } else if (t >= robot.seg.t1 - 1e-9) {
      const t1 = robot.seg.t1;
      if (robot.state === "down") startSeg("grip", t1, TIMES.grip, { k: nextPick });
      else if (robot.state === "grip" || robot.state === "holdB") {
        if (robot.state === "grip") { held = nextPick; cartons[held].knots.push([t1, front[held]]); cartons[held].pick = t1; }
        // place-zone interlock on belt B: the previous carton must have left the drop zone before the load comes down
        const prev = held > 0 ? cartons[held - 1] : null, descend = MOVES.transfer.zDown;
        const clearAt = prev && prev.bAt != null ? prev.bAt + (CARTON.y + PLACE_GAP) / BELTS.B.v - descend : -Infinity;
        if (clearAt > t1 + 1e-9) startSeg("holdB", t1, clearAt - t1, { k: held });
        else startSeg("transfer", t1, MOVES.transfer.T, { move: "transfer", k: held });
      } else if (robot.state === "transfer") startSeg("release", t1, TIMES.release, { k: held });
      else if (robot.state === "release") {
        cartons[held].place = t1; cartons[held].bAt = t1; held = -1; nextPick += 1;
        const ready = settleTime(nextPick);
        if (PARAMS.smart && ready != null && ready <= t1 + MOVES.backPick.descendAt) startSeg("backPick", t1, MOVES.backPick.T, { move: "backPick", k: nextPick });
        else startSeg("back", t1, MOVES.back.T, { move: "back" });
      } else if (robot.state === "back") startSeg("wait", t1, 0);
      else if (robot.state === "backPick") startSeg("grip", t1, TIMES.grip, { k: nextPick });
    }
    // smart-cycle guard: the descent onto the infeed may start only with the photo-eye confirmed
    if (robot.state === "backPick" && !robot.seg.checked && t >= robot.t0 + MOVES.backPick.descendAt - 1e-9) {
      robot.seg.checked = true;
      if (!(sensorOn >= 0 && t - sensorOn >= TIMES.settle - 1e-6)) violations.push({ t, what: "descent without carton at the stopper" });
    }
  }
  const tEnd = t;
  for (const c of cartons) if (c.knots.length) c.knots.push([Math.max(tEnd, c.knots[c.knots.length - 1][0]), c.knots[c.knots.length - 1][1]]);
  DURATION = tEnd + 1.5;
  for (const c of cartons) if (c.bAt != null) c.gone = c.bAt + (BELTS.B.y1 - CARTON.y / 2) / BELTS.B.v;
  segs.push({ kind: "end", t0: tEnd, t1: DURATION });
}

function segAt(t) {
  let lo = 0, hi = segs.length - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (segs[m].t0 <= t) lo = m; else hi = m - 1; }
  return segs[lo];
}
function robotTcp(t) {
  if (PHYS) { const [i, j, a] = physIndex(t); return [lerp(PHYS.tcp[i][0], PHYS.tcp[j][0], a), lerp(PHYS.tcp[i][1], PHYS.tcp[j][1], a)]; }
  const s = segAt(t);
  if (s.move) return MOVES[s.move].at(t - s.t0);
  if (s.kind === "grip" || s.kind === "holdB") return P.pick.slice();
  if (s.kind === "release") return P.place.slice();
  return P.ready.slice();
}
function vacuumOn(t) { if (PHYS) return PHYS.vac[physIndex(t)[0]] === 1; const s = segAt(t); return s.kind === "grip" ? (t - s.t0) / (s.t1 - s.t0) > 0.3 : s.kind === "transfer" || s.kind === "holdB" || s.kind === "release" && (t - s.t0) / (s.t1 - s.t0) < 0.4; }
// knot interpolation: between knots the carton either moves at belt speed or stands still, so linear interpolation is exact
function frontAt(c, t) {
  const K = c.knots;
  if (!K.length || t < K[0][0]) return null;
  for (let i = K.length - 1; i >= 0; i--) {
    if (K[i][0] <= t) {
      if (i === K.length - 1) return K[i][1];
      const [ta, ya] = K[i], [tb, yb] = K[i + 1];
      return lerp(ya, yb, (t - ta) / Math.max(1e-9, tb - ta));
    }
  }
  return null;
}
// carton bottom-face centre [x, y, z] or null when not in the cell
function cartonAt(c, t) {
  if (PHYS) { const q = physCarton(c.k, t); return q ? [q[0], q[1], q[2] - CARTON.h / 2] : null; }
  if (c.spawn == null || t < c.spawn) return null;
  if (c.bAt != null && t >= c.bAt) {
    if (t >= c.gone) return null;
    return [BELTS.B.x, (t - c.bAt) * BELTS.B.v, 0];
  }
  if (c.pick != null && t >= c.pick) { const p = robotTcp(t); return [p[0], 0, p[1] - CARTON.h]; }
  const f = frontAt(c, t);
  return f == null ? null : [BELTS.A.x, f - CARTON.y / 2, 0];
}
const cartonBox = (p) => ({ min: [p[0] - CARTON.x / 2, p[1] - CARTON.y / 2, p[2]], max: [p[0] + CARTON.x / 2, p[1] + CARTON.y / 2, p[2] + CARTON.h] });

// ---------------------------------------------------------------- static obstacles (axis-aligned boxes)
function guideBoxes() {
  const out = [];
  for (const [name, B] of Object.entries(BELTS)) for (const s of [-1, 1]) {
    const x = B.x + s * (B.w / 2 + GUIDE.t / 2);
    out.push({ name: `guide ${name}${s < 0 ? "-" : "+"}`, min: [x - GUIDE.t / 2, B.y0, 0.010], max: [x + GUIDE.t / 2, B.y1, GUIDE.top] });
  }
  out.push({ name: "stopper", min: [BELTS.A.x - BELTS.A.w / 2, STOP_Y, 0.004], max: [BELTS.A.x + BELTS.A.w / 2, STOP_Y + 0.012, 0.070] });
  return out;
}
const STATIC = guideBoxes();
function boxGap(a, b) {      // separation distance between two AABBs (0 if touching or overlapping), and overlap depth
  let d2 = 0, pen = Infinity;
  for (let i = 0; i < 3; i++) {
    const g = Math.max(a.min[i] - b.max[i], b.min[i] - a.max[i]);
    if (g > 0) d2 += g * g; else pen = Math.min(pen, -g);
  }
  return d2 > 0 ? Math.sqrt(d2) : -pen;
}
function pointBoxDist(p, b) {
  let d2 = 0;
  for (let i = 0; i < 3; i++) { const g = Math.max(b.min[i] - p[i], 0, p[i] - b.max[i]); d2 += g * g; }
  return Math.sqrt(d2);
}
// robot envelopes: lower arms (r 32 mm), upper arms (r 45 mm), plate, gripper; y half-width from the CAD
function robotEnvelope(pose) {
  const caps = [];
  pose.elbows.forEach((E, i) => {
    caps.push({ name: "lower arm", a: [E[0], 0, E[1]], b: [pose.joints[i][0], 0, pose.joints[i][1]], r: 0.035 });
    caps.push({ name: "upper arm", a: [pose.motors[i][0], 0, pose.motors[i][1]], b: [E[0], 0, E[1]], r: 0.050 });
  });
  const A = pose.motors[1], E = pose.elbows[1], J = pose.joints[1], o1 = ROBOT.o1, o2 = ROBOT.o2;
  caps.push({ name: "rod pair 1", a: [A[0] + o1[0], 0, A[1] + o1[1]], b: [E[0] + o1[0], 0, E[1] + o1[1]], r: 0.025 });
  caps.push({ name: "rod pair 2", a: [E[0] + o2[0], 0, E[1] + o2[1]], b: [J[0] + o2[0], 0, J[1] + o2[1]], r: 0.020 });
  const f = pose.flange, t = pose.tcp;
  const boxes = [
    { name: "plate", min: [f[0] - 0.13, -0.09, f[1]], max: [f[0] + 0.13, 0.09, f[1] + 0.10] },
    { name: "gripper", min: [t[0] - TOOL.padX / 2, -TOOL.padY / 2, t[1]], max: [t[0] + TOOL.padX / 2, TOOL.padY / 2, f[1]] },
  ];
  return { caps, boxes };
}
function capsuleBoxDist(c, b, n = 24) {
  let m = Infinity;
  for (let i = 0; i <= n; i++) {
    const s = i / n, p = [lerp(c.a[0], c.b[0], s), lerp(c.a[1], c.b[1], s), lerp(c.a[2], c.b[2], s)];
    m = Math.min(m, pointBoxDist(p, b) - c.r);
  }
  return m;
}

// ---------------------------------------------------------------- sampled cycle, dynamics and checks
const RATE = 60;
let S = null;
function sampleCycle() {
  const n = Math.floor(DURATION * RATE) + 1, dt = 1 / RATE;
  const tcp = [], th = [], clear = [], gripPen = [], overlap = [], torque = [], speed = [], acc = [], mu = [], cond = [];
  const dyn = [], acc3 = [];
  for (let i = 0; i < n; i++) tcp.push(robotTcp(i * dt));
  for (let i = 0; i < n; i++) {
    const t = i * dt, p = tcp[i], pose = armPose(p);
    th.push(pose ? pose.th : null);
    const pm = tcp[Math.max(0, i - 1)], pp = tcp[Math.min(n - 1, i + 1)];
    const v = [(pp[0] - pm[0]) / (2 * dt), (pp[1] - pm[1]) / (2 * dt)];
    const a = [(pp[0] - 2 * p[0] + pm[0]) / (dt * dt), (pp[1] - 2 * p[1] + pm[1]) / (dt * dt)];
    speed.push(Math.hypot(...v)); acc.push(Math.hypot(...a)); acc3.push(a);
    if (!pose) { torque.push([NaN, NaN]); dyn.push(null); clear.push({ m: -1, what: "IK" }); gripPen.push(0); overlap.push(0); mu.push([0, 0]); cond.push(Infinity); continue; }
    mu.push(transmission(pose));
    const J = jacobian(pose.th); cond.push(cond2(J));
    const holding = cartons.some((c) => c.pick != null && t >= c.pick && t < c.bAt);
    const d = PHYS ? physDynamics(t) : jointDynamics(t, holding);   // arm-side torque, speed, accel (11-dynamics / measured)
    dyn.push(d); torque.push(d.tau);
    // clearances
    let best = { m: Infinity, what: "" }, pen = 0;
    const note = (d, what) => { if (d < best.m) best = { m: d, what }; };
    const env = robotEnvelope(pose);
    const boxes = cartons.map((c) => { const q = cartonAt(c, t); return q ? { c, q, box: cartonBox(q), held: c.pick != null && t >= c.pick && t < c.bAt } : null; }).filter(Boolean);
    const loose = boxes.filter((b) => !b.held), heldBox = boxes.find((b) => b.held);
    for (const o of STATIC) {
      for (const cp of env.caps) note(capsuleBoxDist(cp, o), `${cp.name} ↔ ${o.name}`);
      for (const bx of env.boxes) note(boxGap(bx, o), `${bx.name} ↔ ${o.name}`);
      if (heldBox && !(o.name === "stopper" && Math.abs(heldBox.q[0] - BELTS.A.x) < (BELTS.A.w - CARTON.x) / 2 + 1e-6)) note(boxGap(heldBox.box, o), `held carton ↔ ${o.name}`);
    }
    for (const lb of loose) {
      for (const cp of env.caps) note(capsuleBoxDist(cp, lb.box), `${cp.name} ↔ carton ${lb.c.k + 1}`);
      if (heldBox) note(boxGap(heldBox.box, lb.box), `held carton ↔ carton ${lb.c.k + 1}`);
      for (const bx of env.boxes) {
        const g = boxGap(bx, lb.box);
        if (bx.name === "plate") note(g, `plate ↔ carton ${lb.c.k + 1}`);
        else pen = Math.max(pen, -g);
      }
    }
    clear.push(best); gripPen.push(pen);
    // carton-carton overlap (any two, any belt): depth in metres, 0 if none (touching in the queue is allowed)
    let ov = 0;
    for (let x = 0; x < boxes.length; x++) for (let y = x + 1; y < boxes.length; y++) ov = Math.max(ov, -boxGap(boxes[x].box, boxes[y].box));
    overlap.push(ov);
  }
  S = { n, dt, tcp, th, clear, gripPen, overlap, torque, speed, acc, acc3, mu, cond, dyn };
  return S;
}

function cycleStats() {
  const placed = cartons.filter((c) => c.place != null).sort((a, b) => a.place - b.place);
  const gaps = placed.slice(1).map((c, i) => c.place - placed[i].place);
  const steady = gaps.length ? gaps.slice(2).reduce((a, b) => a + b, 0) / Math.max(1, gaps.length - 2) : NaN;
  return { placed: placed.length, period: steady, perMin: 60 / steady, pitchB: steady * BELTS.B.v, smart: segs.filter((x) => x.kind === "backPick").length };
}
function rebuild() { applyParams(); schedule(); sampleCycle(); S.motor = motorStats(); S.vac = vacuumStats(); return { DURATION, segs, cartons }; }
