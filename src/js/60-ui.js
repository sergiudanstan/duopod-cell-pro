// ================================================================ UI (everything that depends on the model is rebuilt by refreshModel)
let mode = "program";
let STATS = cycleStats(), minClear = minClearance(), LIMITS = checkLimits();
let lastRow = -1;
const fmt = (v, d = 0) => (Number.isFinite(v) ? v.toFixed(d) : "–");
const svgEsc = (s) => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;");

// panels
const openPanels = { program: new Set(["live-card", "whatif-card", "drives-card"]), manual: new Set(["live-card", "jog-card"]) };
function syncPanels() {
  document.querySelectorAll(".rail-item[data-panel]").forEach((b) => {
    const allowed = b.dataset.modes.split(" ").includes(mode);
    b.hidden = !allowed;
    b.setAttribute("aria-pressed", String(allowed && openPanels[mode].has(b.dataset.panel)));
  });
  document.querySelectorAll(".drawer .card").forEach((c) => { c.hidden = !openPanels[mode].has(c.id); });
  $("drawer").hidden = openPanels[mode].size === 0;
  requestAnimationFrame(resize);
}
document.querySelectorAll(".rail-item[data-panel]").forEach((b) => b.addEventListener("click", () => {
  const s = openPanels[mode]; s.has(b.dataset.panel) ? s.delete(b.dataset.panel) : s.add(b.dataset.panel); syncPanels();
}));
document.querySelectorAll(".card .close").forEach((b) => b.addEventListener("click", () => { openPanels[mode].delete(b.closest(".card").id); syncPanels(); }));
if (matchMedia("(max-width: 900px)").matches) { openPanels.program = new Set(); openPanels.manual = new Set(); }

// ---------------------------------------------------------------- program list
let progSegs = [], programRows = [];
function buildProgram() {
  progSegs = segs.filter((s) => s.kind !== "end" && s.t1 - s.t0 > 1e-6);
  $("program-total").textContent = `${progSegs.length} steps · ${DURATION.toFixed(1)} s`;
  $("program").replaceChildren();
  programRows = progSegs.map((s) => {
    const li = document.createElement("li");
    li.innerHTML = `<i class="fill"></i><span class="chip delta">${s.k != null ? "#" + String(s.k + 1).padStart(2, "0") : "ROBOT"}</span><span class="label"></span><span class="dur">${(s.t1 - s.t0).toFixed(2)} s</span><code></code>`;
    li.querySelector(".label").textContent = LABEL[s.kind];
    li.querySelector(".label").insertAdjacentHTML("beforeend", `<small>${s.t0.toFixed(2)}</small>`);
    li.querySelector("code").textContent = CMD[s.kind]();
    li.addEventListener("click", () => seek(s.t0 + 1e-3));
    $("program").append(li); return li;
  });
  lastRow = -1;
}

// ---------------------------------------------------------------- working range SVG (robot plane, flange, operator view)
const WS = { W: 340, H: 250, s: 250 };
function drawWorkspace() {
  const { W, H, s } = WS, cx = W / 2, z0 = flangeTop + 0.08;
  const X = (x) => cx - x * s, Zp = (z) => 18 + (z0 - z) * s;    // mirrored: as seen from the operator side (+y)
  const ds = dsOutline.pts.map((p) => `${X(p[0]).toFixed(1)},${Zp(p[2]).toFixed(1)}`).join(" ");
  const path = flangePts.filter((_, i) => i % 2 === 0).map((p) => `${X(p[0]).toFixed(1)},${Zp(p[2]).toFixed(1)}`).join(" ");
  const pk = [P.pick, P.place].map((p) => [X(p[0]), Zp(p[1] + TOOL.len)]);
  const belt = (B, col) => `<rect x="${X(B.x + B.w / 2)}" y="${Zp(0)}" width="${B.w * s}" height="5" fill="${col}" opacity="0.6"/>`;
  $("ws").innerHTML = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="Working range and cycle path">
    <polyline points="${ds}" fill="none" stroke="${css("--faint")}" stroke-dasharray="4 3"/>
    <rect x="${X(0.3)}" y="${Zp(flangeTop)}" width="${0.6 * s}" height="${0.4 * s}" fill="${css("--delta-soft")}" stroke="${css("--delta")}"/>
    ${belt(BELTS.A, css("--t3a"))}${belt(BELTS.B, css("--t3b"))}
    <polyline points="${path}" fill="none" stroke="${css("--delta")}" stroke-width="2"/>
    ${pk.map(([x, y], i) => `<circle cx="${x}" cy="${y}" r="4" fill="${css(i ? "--t3b" : "--t3a")}"/>`).join("")}
    <circle id="ws-dot" r="5" fill="${css("--accent")}"/>
    <text x="${X(0.5)}" y="12">1000 × 400 datasheet · operator view</text>
    <text x="${X(0.29)}" y="${Zp(flangeTop) + 26}">600 × 400 required</text>
    <text x="${X(BELTS.A.x) - 14}" y="${Zp(0) + 18}">belt A</text><text x="${X(BELTS.B.x) - 14}" y="${Zp(0) + 18}">belt B</text>
  </svg>`;
}
function moveWsDot(tcp) {
  const d = document.getElementById("ws-dot"); if (!d) return;
  const z0 = flangeTop + 0.08;
  d.setAttribute("cx", (WS.W / 2 - tcp[0] * WS.s).toFixed(1)); d.setAttribute("cy", (18 + (z0 - tcp[1] - TOOL.len) * WS.s).toFixed(1));
}

// ---------------------------------------------------------------- drives: live rows + torque/time and torque/speed charts
const TH_RANGE = (() => {
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i <= 40; i++) for (let j = 0; j <= 16; j++) {
    const q = ik2([-0.5 + i / 40, flangeTop - 0.4 * j / 16 - TOOL.len]); if (!q) continue;
    lo = Math.min(lo, ...q); hi = Math.max(hi, ...q);
  }
  return [lo, hi];
})();
// M1 drives the right arm (index 1), M2 the left arm (index 0)
const MOTOR_ROWS = [["M1", 1, "--delta"], ["M2", 0, "--t3b"]];
const driveRows = MOTOR_ROWS.map(([name, j, col]) => {
  const box = document.createElement("div"); box.className = "drive";
  box.innerHTML = `<h3 style="color:var(${col})">${name} <em></em></h3>
    <div class="joint"><span>arm angle</span><div class="track delta"><i></i></div><span class="v">0°</span></div>
    <div class="joint"><span>torque</span><div class="bar"><i style="background:var(${col})"></i></div><span class="v">0 Nm</span></div>
    <div class="joint"><span>speed</span><div class="bar"><i style="background:var(${col})"></i></div><span class="v">0 rpm</span></div>`;
  $("drives").append(box);
  const [a, tq, sp] = box.querySelectorAll(".joint");
  return { j, em: box.querySelector("em"), ai: a.querySelector("i"), av: a.querySelector(".v"), ti: tq.querySelector("i"), tv: tq.querySelector(".v"), si: sp.querySelector("i"), sv: sp.querySelector(".v") };
});
function showDrives(i, th) {
  const m = S.motor;
  driveRows.forEach((r) => {
    const u = th ? (th[r.j] - TH_RANGE[0]) / (TH_RANGE[1] - TH_RANGE[0]) : 0;
    r.ai.style.left = `${clamp(u, 0, 1) * 100}%`; r.av.textContent = th ? `${(th[r.j] * DEG).toFixed(1)}°` : "–";
    const tq = i != null ? m.tm[i][r.j] : NaN, n = i != null ? m.nm[i][r.j] : NaN;
    r.ti.style.width = `${clamp(Math.abs(tq) / MOTOR.Mmax, 0, 1) * 100}%`; r.tv.textContent = Number.isFinite(tq) ? `${tq.toFixed(2)} Nm` : "–";
    r.si.style.width = `${clamp(Math.abs(n) / MOTOR.nMax, 0, 1) * 100}%`; r.sv.textContent = Number.isFinite(n) ? `${n.toFixed(0)} rpm` : "–";
    r.em.textContent = `peak ${fmt(m.peak[r.j], 2)} · RMS ${fmt(m.rms[r.j], 2)} Nm`;
  });
}
const CH = { W: 330, H: 150, L: 34, R: 8, T: 10, B: 22 };
function drawDriveCharts() {
  const m = S.motor, { W, H, L, R, T, B } = CH;
  const lim = PARAMS.torqueLimit * MOTOR.Mmax;
  const ymax = Math.max(MOTOR.M0 * 1.15, ...m.peak.map((v) => v * 1.15));
  const X = (t) => L + (t / DURATION) * (W - L - R), Y = (v) => T + (1 - (v + ymax) / (2 * ymax)) * (H - T - B);
  const line = (j) => m.tm.map((q, k) => (k % 2 || !Number.isFinite(q[j]) ? null : `${X(k * S.dt).toFixed(1)},${Y(q[j]).toFixed(1)}`)).filter(Boolean).join(" ");
  const hline = (v, cls, label) => (Math.abs(v) <= ymax ? `<line x1="${L}" x2="${W - R}" y1="${Y(v)}" y2="${Y(v)}" class="${cls}"/><text x="${W - R - 2}" y="${Y(v) - 3}" text-anchor="end">${label}</text>` : "");
  $("chart-tq").innerHTML = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="Motor torque over the cycle">
    <line x1="${L}" x2="${W - R}" y1="${Y(0)}" y2="${Y(0)}" class="axis"/>
    ${hline(MOTOR.M0, "lim0", "S1 M0")}${hline(-MOTOR.M0, "lim0", "")}${hline(lim, "lim1", `${(PARAMS.torqueLimit * 100).toFixed(0)} % Mmax`)}${hline(-lim, "lim1", "")}
    ${MOTOR_ROWS.map(([, j, col]) => `<polyline points="${line(j)}" fill="none" stroke="var(${col})" stroke-width="1.3"/>`).join("")}
    <line id="tq-head" x1="${L}" x2="${L}" y1="${T}" y2="${H - B}" stroke="var(--accent)"/>
    <text x="${L - 4}" y="${Y(ymax * 0.85) + 3}" text-anchor="end">${fmt(ymax * 0.85, 0)}</text><text x="${L - 4}" y="${Y(-ymax * 0.85) + 3}" text-anchor="end">${fmt(-ymax * 0.85, 0)}</text>
    <text x="${L}" y="${H - 6}">0 s</text><text x="${W - R}" y="${H - 6}" text-anchor="end">${DURATION.toFixed(1)} s · motor torque, Nm</text>
  </svg>`;
  // torque-speed: |n| vs |tau| on the motor envelope (S1 line between published points, Mmax line, 95 % nmax)
  const nMax = MOTOR.nMax, tMax = MOTOR.Mmax * 1.08;
  const XS = (n) => L + (Math.abs(n) / nMax) * (W - L - R), YS = (v) => T + (1 - Math.abs(v) / tMax) * (H - T - B);
  let dots = "";
  MOTOR_ROWS.forEach(([, j, col]) => { for (let k = 0; k < S.n; k += 3) { const q = m.tm[k][j], n = m.nm[k][j]; if (Number.isFinite(q)) dots += `<circle cx="${XS(n).toFixed(1)}" cy="${YS(q).toFixed(1)}" r="1.4" fill="var(${col})" opacity="0.45"/>`; } });
  const rms = MOTOR_ROWS.map(([, j, col]) => `<rect x="${(XS(m.nMean[j]) - 4).toFixed(1)}" y="${(YS(m.rms[j]) - 4).toFixed(1)}" width="8" height="8" fill="none" stroke="var(${col})" stroke-width="2"/>`).join("");
  $("chart-ns").innerHTML = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="Torque against speed on the motor envelope">
    <polyline points="${XS(0)},${YS(MOTOR.M0)} ${XS(MOTOR.nN)},${YS(MOTOR.Mn)} ${XS(nMax)},${YS(MOTOR.Mn)}" fill="none" class="lim0"/>
    <line x1="${XS(0)}" x2="${XS(nMax)}" y1="${YS(MOTOR.Mmax)}" y2="${YS(MOTOR.Mmax)}" class="lim1"/>
    <line x1="${XS(lim > 0 ? 0 : 0)}" x2="${XS(nMax)}" y1="${YS(lim)}" y2="${YS(lim)}" class="lim2"/>
    <line x1="${XS(nMax * 0.95)}" x2="${XS(nMax * 0.95)}" y1="${T}" y2="${H - B}" class="lim1"/>
    ${dots}${rms}
    <text x="${XS(0) + 2}" y="${YS(MOTOR.Mmax) - 3}">Mmax ${MOTOR.Mmax} Nm</text><text x="${XS(MOTOR.nN * 0.42)}" y="${YS(MOTOR.M0 * 0.62)}">S1 (simplified)</text>
    <text x="${L - 4}" y="${YS(MOTOR.M0) + 3}" text-anchor="end">${MOTOR.M0}</text>
    <text x="${L}" y="${H - 6}">0</text><text x="${W - R}" y="${H - 6}" text-anchor="end">${nMax} rpm · □ RMS at mean speed</text>
  </svg>`;
}
function moveTqHead(t) { const h = document.getElementById("tq-head"); if (!h) return; const x = CH.L + (t / DURATION) * (CH.W - CH.L - CH.R); h.setAttribute("x1", x); h.setAttribute("x2", x); }

// ---------------------------------------------------------------- timeline lanes: robot, belt A escapement, belt B arrivals
const lanes = $("lanes");
const KIND_ALPHA = { down: 0.55, grip: 1, transfer: 0.85, release: 1, back: 0.45, backPick: 0.6, holdB: 0.25 };
function drawLanes() {
  const w = lanes.clientWidth, h = lanes.clientHeight; if (!w) return;
  const dpr = Math.min(devicePixelRatio, 2); lanes.width = w * dpr; lanes.height = h * dpr;
  const g = lanes.getContext("2d"); g.scale(dpr, dpr); g.clearRect(0, 0, w, h);
  const X = (t) => (t / DURATION) * w;
  [4, 21, 38].forEach((y) => { g.fillStyle = css("--line"); g.fillRect(0, y, w, 11); });
  g.fillStyle = css("--delta");
  for (const s of segs) if (KIND_ALPHA[s.kind]) { g.globalAlpha = KIND_ALPHA[s.kind]; g.fillRect(X(s.t0), 4, Math.max(1, X(s.t1) - X(s.t0)), 11); }
  g.globalAlpha = 0.85; g.fillStyle = css("--t3a");
  let open = null;
  for (const e of escape) { if (e.ev === "release") open = e.t; else if (open != null) { g.fillRect(X(open), 21, Math.max(1, X(e.t) - X(open)), 11); open = null; } }
  g.fillStyle = css("--t3b");
  for (const c of cartons) if (c.bAt != null) g.fillRect(X(c.bAt), 38, Math.max(2, X(Math.min(DURATION, c.bAt + CARTON.y / BELTS.B.v)) - X(c.bAt)), 11);
  g.globalAlpha = 1;
}

// ---------------------------------------------------------------- report (drawer)
function verdict(ok, text) { return `<span class="${ok ? "ok" : "warn"}">${ok ? "✓" : "✗"} ${text}</span>`; }
function reportRows() {
  const m = S.motor, v = S.vac;
  const moving = 2 * (BODY_REF.upper.m + BODY_REF.lower.m) + BODY_REF.plate.m + BODY_REF.tri.m + BODY_REF.rod1.m + BODY_REF.rod2.m;
  return [
    ["Robot", "Autonox DuoPod RVS2-1000-3kg · 2 DOF · ceiling"],
    ["Arms L1 / L2", `${ROBOT.L1 * 1000} / ${ROBOT.L2 * 1000} mm (STEP bores)`],
    ["Moving mass", `${fmt(moving, 1)} kg arms + plate + linkage (STEP, conservative)`],
    ["Range", "1000 × 400 datasheet · 600 × 400 required ✓"],
    ["Payload", `${PARAMS.payload} kg carton + ${TOOL.mass} kg gripper`],
    ["Cycle", `${fmt(STATS.period, 3)} s → ${fmt(STATS.perMin, 1)} /min${PARAMS.smart ? ` · ${STATS.smart} direct returns` : ""}`],
    ["Belt B pitch", `${fmt(STATS.pitchB * 1000)} mm at ${BELTS.B.v.toFixed(2)} m/s`],
    ["Peak TCP", `${fmt(Math.max(...S.speed), 2)} m/s · ${fmt(Math.max(...S.acc), 1)} m/s² (${fmt(Math.max(...S.acc) / G, 1)} g)`],
    ["Arm torque peak", `M1 ${fmt(m.armPeak[1])} · M2 ${fmt(m.armPeak[0])} Nm`],
    ["Motor", `${MOTOR.name} · i = ${PARAMS.ratio} (assumed) · η ${PARAMS.eta}`],
    ["Motor peak", verdict(m.ok.peak, `${fmt(Math.max(...m.peak), 2)} / ${fmt(PARAMS.torqueLimit * MOTOR.Mmax, 1)} Nm`)],
    ["Motor RMS", verdict(m.ok.rms, `${fmt(Math.max(...m.rms), 2)} / ${fmt(Math.min(...m.cont), 1)} Nm (S1)`)],
    ["Motor speed", verdict(m.ok.speed, `${fmt(Math.max(...m.nPeak))} / ${fmt(0.95 * MOTOR.nMax)} rpm`)],
    ["Vacuum", verdict(v.ok, `SF ${fmt(v.sf, 2)} · ${fmt(v.avail)} N at ${VAC.dp / 1000} kPa`)],
    ["Min clearance", verdict(minClear.m >= 0.01, `${fmt(minClear.m * 1000, 1)} mm · ${svgEsc(minClear.what)}`)],
  ];
}
function renderReport() { $("report").innerHTML = reportRows().map(([k, val]) => `<dt>${k}</dt><dd>${val}</dd>`).join(""); }
function showTcp(tcp) {
  $("tcp").innerHTML = `<span class="small-note" style="margin:0">TCP</span>` +
    [["x", tcp[0] * 1000], ["z", tcp[1] * 1000], ["flange z", (tcp[1] + TOOL.len) * 1000]].map(([k, v]) => `<div><small>${k}</small><b>${v.toFixed(1)}</b></div>`).join("");
}
$("copy-csv").addEventListener("click", async () => {
  const lines = ["t_s,x_mm,z_mm,thM2_deg,thM1_deg,tauArmM2_Nm,tauArmM1_Nm,tauMotM2_Nm,tauMotM1_Nm,nM2_rpm,nM1_rpm"];
  for (let i = 0; i < S.n; i++) {
    const p = S.tcp[i], th = S.th[i] || [NaN, NaN], tq = S.torque[i], tm = S.motor.tm[i], n = S.motor.nm[i];
    lines.push([(i * S.dt).toFixed(4), (p[0] * 1000).toFixed(2), (p[1] * 1000).toFixed(2), (th[0] * DEG).toFixed(3), (th[1] * DEG).toFixed(3),
      tq[0].toFixed(2), tq[1].toFixed(2), tm[0].toFixed(3), tm[1].toFixed(3), n[0].toFixed(0), n[1].toFixed(0)].join(","));
  }
  try { await navigator.clipboard.writeText(lines.join("\n")); $("copy-note").textContent = `Copied ${S.n} rows.`; }
  catch { $("copy-note").textContent = "Clipboard not available in this browser."; }
});

// ---------------------------------------------------------------- jog (manual mode): TCP x / z in the robot plane
const jog = { x: P.ready[0], z: P.ready[1], vac: false };
const jogDefs = [
  { key: "x", label: "x mm", min: -500, max: 500 },
  { key: "z", label: "z mm", min: Math.round((flangeTop - 0.51 - TOOL.len) * 1000), max: Math.round((flangeTop - TOOL.len) * 1000) },
];
const jogRows = jogDefs.map((d) => {
  const row = document.createElement("div"); row.className = "jog-row delta";
  row.innerHTML = `<label>${d.label}</label><input type="range" min="${d.min}" max="${d.max}" step="1"><input type="number" min="${d.min}" max="${d.max}" step="1">`;
  const [r, n] = row.querySelectorAll("input");
  const set = (v) => { const q = clamp(Number(v), d.min, d.max); const prev = jog[d.key]; jog[d.key] = q / 1000; if (!ik2([jog.x, jog.z])) jog[d.key] = prev; syncJog(); };
  r.addEventListener("input", () => set(r.value)); n.addEventListener("change", () => set(n.value));
  $("jog").append(row); return { d, r, n };
});
function syncJog() {
  for (const { d, r, n } of jogRows) { r.value = n.value = Math.round(jog[d.key] * 1000); }
  const q = ik2([jog.x, jog.z]);
  if (!q) { $("jog-note").textContent = "out of reach"; return; }
  const ts = staticTorque([jog.x, jog.z], jog.vac);
  $("jog-note").textContent = `θ M1 ${(q[1] * DEG).toFixed(1)}° · θ M2 ${(q[0] * DEG).toFixed(1)}° · static arm torque M1 ${ts[1].toFixed(0)} / M2 ${ts[0].toFixed(0)} Nm`;
}
$("jog-vac").addEventListener("change", (e) => { jog.vac = e.target.checked; syncJog(); });
$("jog-home").addEventListener("click", () => { [jog.x, jog.z] = P.ready; syncJog(); });
$("jog-pick").addEventListener("click", () => { [jog.x, jog.z] = P.pick; syncJog(); });
$("jog-place").addEventListener("click", () => { [jog.x, jog.z] = P.place; syncJog(); });
syncJog();

// ---------------------------------------------------------------- refresh after a rebuild (what-if)
function refreshCharts() { drawLanes(); drawWorkspace(); drawDriveCharts(); }
function refreshModel() {
  STATS = cycleStats(); minClear = minClearance(); LIMITS = checkLimits();
  buildProgram(); rebuildPaths(); refreshCharts(); renderReport(); renderWhatIfKpi(); renderPlc(); buildPrintReport();
  $("clock-total").textContent = DURATION.toFixed(1);
  $("timeline").setAttribute("aria-valuemax", DURATION.toFixed(1));
  if (typeof time !== "undefined") time = Math.min(time, DURATION);
}
