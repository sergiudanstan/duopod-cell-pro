// ---------------------------------------------------------------- what-if: live parameters, optimizer, verdicts
const WHATIF_DEFS = [
  ["vA", "Belt A speed", (v) => `${(v * 1000).toFixed(0)} mm/s`, 1],
  ["vB", "Belt B speed", (v) => `${(v * 1000).toFixed(0)} mm/s`, 1],
  ["payload", "Payload", (v) => `${v.toFixed(1)} kg`, 1],
  ["travel", "Travel height (TCP)", (v) => `${(v * 1000).toFixed(0)} mm`, 1],
  ["ratio", "Gearbox ratio i (assumed)", (v) => `1 : ${v.toFixed(0)}`, 1],
  ["scale", "Motion time scale", (v) => `× ${v.toFixed(2)}`, 1],
  ["torqueLimit", "Motor peak limit", (v) => `${(v * 100).toFixed(0)} % Mmax`, 1],
];
const whatifRows = WHATIF_DEFS.map(([key, label, show]) => {
  const [lo, hi, step] = PARAM_RANGE[key];
  const row = document.createElement("label"); row.className = "wi-row";
  row.innerHTML = `<span>${label}</span><input type="range" min="${lo}" max="${hi}" step="${step}" aria-label="${label}"><b class="mono"></b>`;
  const input = row.querySelector("input"), out = row.querySelector("b");
  input.addEventListener("input", () => { PARAMS[key] = Number(input.value); out.textContent = show(PARAMS[key]); queueRebuild(); });
  $("whatif").append(row);
  return { key, input, out, show };
});
const smartBox = $("wi-smart");
smartBox.addEventListener("change", () => { PARAMS.smart = smartBox.checked; queueRebuild(); });
function syncWhatIfInputs() {
  for (const r of whatifRows) { r.input.value = PARAMS[r.key]; r.out.textContent = r.show(PARAMS[r.key]); }
  smartBox.checked = PARAMS.smart;
}
let rebuildTimer = 0;
function queueRebuild() {
  clearTimeout(rebuildTimer);
  rebuildTimer = setTimeout(() => { rebuild(); refreshModel(); }, 90);
}
$("wi-reset").addEventListener("click", () => { Object.assign(PARAMS, PARAMS0); syncWhatIfInputs(); rebuild(); refreshModel(); $("wi-opt-note").textContent = ""; });
$("wi-opt").addEventListener("click", () => {
  $("wi-opt-note").textContent = "Optimizing…";
  setTimeout(() => {
    const t0 = performance.now(), o = optimizeScale();
    syncWhatIfInputs(); refreshModel();
    $("wi-opt-note").textContent = o.scale == null
      ? `No feasible time scale in range: ${o.binding}.`
      : `Fastest scale ×${o.scale.toFixed(3)} → ${fmt(STATS.perMin, 1)} /min. Binding limit: ${o.binding}. (${(performance.now() - t0).toFixed(0)} ms, ${9 + 2} rebuilds)`;
  }, 20);
});
function renderWhatIfKpi() {
  const m = S.motor, v = S.vac;
  const kpi = [
    ["Cadence", `${fmt(STATS.perMin, 1)}`, "/min", `period ${fmt(STATS.period, 2)} s`],
    ["Motor peak", `${fmt(Math.max(...m.peak), 1)}`, "Nm", `limit ${fmt(PARAMS.torqueLimit * MOTOR.Mmax, 1)}`],
    ["Motor RMS", `${fmt(Math.max(...m.rms), 2)}`, "Nm", `S1 ${fmt(Math.min(...m.cont), 1)}`],
    ["Motor speed", `${fmt(Math.max(...m.nPeak))}`, "rpm", `max ${MOTOR.nMax}`],
    ["Vacuum SF", `${fmt(v.sf, 2)}`, "", `need ≥ ${VAC.sfReq}`],
    ["Clearance", `${fmt(minClear.m * 1000, 0)}`, "mm", "need ≥ 10"],
  ];
  $("wi-kpi").innerHTML = kpi.map(([k, val, u, sub]) => `<div class="stat"><small>${k}</small><div class="value">${val}<em>${u}</em></div><div class="sub">${sub}</div></div>`).join("");
  $("wi-verdict").innerHTML = LIMITS.length
    ? `<span class="verdict fail">✗ ${LIMITS.length} limit${LIMITS.length > 1 ? "s" : ""}</span> ${LIMITS.map(svgEsc).join(" · ")}`
    : `<span class="verdict pass">✓ all limits hold</span> reach, overlaps, clearance, interlocks, motor peak/RMS/speed, vacuum`;
}
syncWhatIfInputs();
