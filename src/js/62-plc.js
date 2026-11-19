// ---------------------------------------------------------------- PLC: I/O list, Structured Text generated from the simulated logic, HMI pages, print report
const IO = [
  ["DI", "%I0.0", "B1", "Carton at end stop (retro-reflective photo-eye, belt A)"],
  ["DI", "%I0.1", "B2", "Vacuum OK (pressure switch on the ejector)"],
  ["DI", "%I0.2", "B3", "Belt B drop zone clear (photo-eye)"],
  ["DI", "%I0.3", "B4", "Escapement blade up (reed)"],
  ["DI", "%I0.4", "B5", "Escapement blade down (reed)"],
  ["DI", "%I0.5", "B6", "Side clamp open (reed)"],
  ["DI", "%I0.6", "S1", "Guard doors closed (safety switch, via safety relay)"],
  ["DI", "%I0.7", "F1", "Light curtains belt A / B clear (via safety relay)"],
  ["DI", "%I1.0", "S2", "E-stop chain OK"],
  ["DI", "%I1.1", "B7", "Compressed air ≥ 5.5 bar"],
  ["DO", "%Q0.0", "Y1", "Vacuum ejector ON"],
  ["DO", "%Q0.1", "Y2", "Blow-off pulse"],
  ["DO", "%Q0.2", "Y3", "Escapement blade (ON = up / closed)"],
  ["DO", "%Q0.3", "Y4", "Side clamp (ON = clamp)"],
  ["DO", "%Q0.4", "K1", "Belt A run"],
  ["DO", "%Q0.5", "K2", "Belt B run"],
  ["DO", "%Q0.6", "H1", "Stack light green"],
  ["DO", "%Q0.7", "H2", "Stack light amber"],
  ["DO", "%Q1.0", "H3", "Stack light red"],
  ["AX", "Axis 1", "M1", `${MOTOR.name}, right arm, EtherCAT`],
  ["AX", "Axis 2", "M2", `${MOTOR.name}, left arm, EtherCAT`],
];
function plcText() {
  const mm = (v) => (v * 1000).toFixed(0), s = (v) => v.toFixed(3);
  return `(* DuoPod transfer cell - generated from the simulation constants, ${new Date().toISOString().slice(0, 10)} *)
PROGRAM PRG_Transfer
VAR
    eState      : (WAIT_CARTON, DESCEND, GRIP, HOLD_B, TRANSFER, RELEASE, RETURN, RETURN_PICK) := WAIT_CARTON;
    tSettle     : TON;   (* carton settled at the end stop *)
    tGrip       : TON;   (* vacuum build-up *)
    tRelease    : TON;   (* blow-off *)
    rPickX      : LREAL := ${mm(P.pick[0])};    (* mm, robot plane *)
    rPickZ      : LREAL := ${mm(Z.pick)};
    rPlaceX     : LREAL := ${mm(P.place[0])};
    rPlaceZ     : LREAL := ${mm(Z.place)};
    rTravelZ    : LREAL := ${mm(Z.travel)};
    rClearZ     : LREAL := ${mm(Z.clear)};      (* held carton above the next one *)
    rColumnHalf : LREAL := ${mm((CARTON.x + TOOL.padX) / 2 + 0.02)};
    bSmartCycle : BOOL  := ${PARAMS.smart ? "TRUE" : "FALSE"};
    bReleaseOK  : BOOL;
    bColumnClear: BOOL;
END_VAR

(* ---- escapement interlock: release the queue head only when the stop zone is empty
        and nothing of the robot or its load is inside the pick column below rClearZ *)
bColumnClear := (ABS(fbRobot.rTcpX - rPickX) > rColumnHalf) OR (fbRobot.rTcpZ >= rClearZ);
bReleaseOK   := bHeadAtBlade AND NOT bCartonInStopZone AND bColumnClear AND bSafetyOK;
IF bReleaseOK AND Y3_BladeUp THEN
    Y3_BladeUp := FALSE;                  (* blade down: head carton runs to the end stop *)
    Y4_Clamp   := bNextCartonTouching;    (* hold the carton behind it *)
ELSIF NOT Y3_BladeUp AND bHeadRearPastBlade THEN
    Y3_BladeUp := TRUE;  Y4_Clamp := FALSE;
END_IF

tSettle(IN := B1_CartonAtStop, PT := T#${Math.round(TIMES.settle * 1000)}MS);

CASE eState OF
WAIT_CARTON:
    IF tSettle.Q AND bSafetyOK THEN
        fbRobot.MoveL(rPickX, rPickZ, T := ${s(MOVES.down.T)});           eState := DESCEND;
    END_IF
DESCEND:
    IF fbRobot.Done THEN Y1_Vacuum := TRUE; eState := GRIP; END_IF
GRIP:
    tGrip(IN := TRUE, PT := T#${Math.round(TIMES.grip * 1000)}MS);
    IF tGrip.Q AND B2_VacuumOK THEN tGrip(IN := FALSE); eState := HOLD_B; END_IF
HOLD_B:   (* place-zone interlock: previous carton ${mm(CARTON.y + PLACE_GAP)} mm downstream before the load comes down *)
    IF B3_DropZoneClear THEN
        fbRobot.GoalPost(rPickX, rPickZ, rPlaceX, rPlaceZ, rTravelZ, T := ${s(MOVES.transfer.T)});   eState := TRANSFER;
    END_IF
TRANSFER:
    IF fbRobot.Done THEN Y1_Vacuum := FALSE; Y2_BlowOff := TRUE; eState := RELEASE; END_IF
RELEASE:
    tRelease(IN := TRUE, PT := T#${Math.round(TIMES.release * 1000)}MS);
    IF tRelease.Q THEN
        tRelease(IN := FALSE); Y2_BlowOff := FALSE;
        IF bSmartCycle AND fbBeltA.CartonSettledBy(fbRobot.Now + ${s(MOVES.backPick.descendAt)}) THEN
            fbRobot.GoalPost(rPlaceX, rPlaceZ, rPickX, rPickZ, rTravelZ, T := ${s(MOVES.backPick.T)});  eState := RETURN_PICK;
        ELSE
            fbRobot.MoveL2(rPlaceX, rTravelZ, rPickX, rTravelZ, T := ${s(MOVES.back.T)});        eState := RETURN;
        END_IF
    END_IF
RETURN_PICK:   (* the descent may start only with B1 confirmed; otherwise the move stops at rTravelZ *)
    IF fbRobot.AtPhase(DESCENT) AND NOT tSettle.Q THEN fbRobot.Halt(); eState := WAIT_CARTON; END_IF
    IF fbRobot.Done THEN Y1_Vacuum := TRUE; eState := GRIP; END_IF
RETURN:
    IF fbRobot.Done THEN eState := WAIT_CARTON; END_IF
END_CASE

(* ---- stack light *)
H1_Green := bAuto AND bSafetyOK;  H2_Amber := NOT bAuto;  H3_Red := NOT bSafetyOK OR bAlarm;
END_PROGRAM`;
}
function renderPlc() {
  $("io-list").innerHTML = `<table><thead><tr><th>Type</th><th>Address</th><th>Tag</th><th>Function</th></tr></thead><tbody>${IO.map((r) => `<tr>${r.map((c) => `<td>${svgEsc(c)}</td>`).join("")}</tr>`).join("")}</tbody></table>`;
  $("plc-code").textContent = plcText();
}
$("copy-plc").addEventListener("click", async () => {
  try { await navigator.clipboard.writeText(plcText()); $("plc-note").textContent = "Copied the Structured Text."; }
  catch { $("plc-note").textContent = "Clipboard not available in this browser."; }
});

// ---------------------------------------------------------------- HMI pages on the 3D panel (rotate every 3 s while running)
const HMI_PAGES = ["OVERVIEW", "DRIVES", "CONVEYORS", "ALARMS"];
let hmiAcc = 0.5, hmiPageT = 0;
function paintHmi(dt, ctx) {
  hmiAcc += dt; hmiPageT += dt; if (hmiAcc < 0.25) return; hmiAcc = 0;
  const page = HMI_PAGES[Math.floor(hmiPageT / 3) % HMI_PAGES.length];
  const cv = hmiScreen.canvas, g = cv.getContext("2d"), W = cv.width;
  g.fillStyle = "#11161c"; g.fillRect(0, 0, W, cv.height);
  g.fillStyle = "#1d2630"; g.fillRect(0, 0, W, 52);
  g.fillStyle = "#8fd0c8"; g.font = "600 26px sans-serif"; g.fillText("TRANSFER CELL", 18, 35);
  HMI_PAGES.forEach((p, k) => { g.fillStyle = p === page ? "#8fd0c8" : "#5d6b78"; g.font = "600 15px sans-serif"; g.fillText(p, 230 + k * 72, 33); });
  const hold = !ctx.running;
  g.fillStyle = hold ? "#e0b15a" : "#7ddea0"; g.font = "700 40px sans-serif"; g.fillText(hold ? "HOLD" : "AUTO", 18, 104);
  g.fillStyle = "#d7dee6"; g.font = "500 23px sans-serif";
  const lines = (arr) => arr.forEach((line, k) => g.fillText(line, 18, 150 + k * 38));
  const bar = (y, frac, col) => { g.fillStyle = "#2a333d"; g.fillRect(250, y - 18, 230, 20); g.fillStyle = col; g.fillRect(250, y - 18, 230 * clamp(frac, 0, 1), 20); };
  if (page === "OVERVIEW") lines([`Placed     ${ctx.done} / ${N_CARTONS}`, `Rate       ${fmt(STATS.perMin, 1)} /min`, `Cycle      ${fmt(STATS.period, 2)} s`, `Clearance  ${fmt(ctx.clear * 1000)} mm`, `Step       ${LABEL[ctx.seg.kind]}`]);
  else if (page === "DRIVES") {
    lines([`M1 torque  ${fmt(ctx.tm ? ctx.tm[1] : NaN, 2)} Nm`, `M1 speed   ${fmt(ctx.nm ? Math.abs(ctx.nm[1]) : NaN)} rpm`, `M2 torque  ${fmt(ctx.tm ? ctx.tm[0] : NaN, 2)} Nm`, `M2 speed   ${fmt(ctx.nm ? Math.abs(ctx.nm[0]) : NaN)} rpm`, `i = ${PARAMS.ratio}   peak ${fmt(Math.max(...S.motor.peak), 1)} Nm`]);
    if (ctx.tm) { bar(150, Math.abs(ctx.tm[1]) / MOTOR.Mmax, "#3cc7c3"); bar(226, Math.abs(ctx.tm[0]) / MOTOR.Mmax, "#d29af0"); }
  } else if (page === "CONVEYORS") lines([`Belt A     ${(BELTS.A.v * 1000).toFixed(0)} mm/s`, `Queue A    ${ctx.queue}`, `Blade      ${ctx.esc.open ? "DOWN" : "UP"}`, `Clamp      ${ctx.esc.clampOn ? "ON" : "OFF"}`, `Belt B     ${(BELTS.B.v * 1000).toFixed(0)} mm/s`]);
  else {
    const al = LIMITS.length ? LIMITS : ["No active alarms"];
    g.fillStyle = LIMITS.length ? "#ff8a80" : "#7ddea0"; al.slice(0, 5).forEach((a, k) => g.fillText((LIMITS.length ? "▲ " : "● ") + a, 18, 150 + k * 38));
  }
  hmiScreen.texture.needsUpdate = true;
}

// ---------------------------------------------------------------- print report (browser "Save as PDF")
function buildPrintReport() {
  const r = $("print-report");
  const svg = (id) => document.getElementById(id)?.innerHTML || "";
  r.innerHTML = `<h1>DuoPod transfer cell — engineering report</h1>
    <p class="meta">Generated ${new Date().toLocaleString()} from the live simulation · parameters: belt A ${BELTS.A.v.toFixed(2)} m/s, belt B ${BELTS.B.v.toFixed(2)} m/s, payload ${PARAMS.payload} kg, travel ${(Z.travel * 1000).toFixed(0)} mm, i = ${PARAMS.ratio}, time scale ×${PARAMS.scale.toFixed(2)}, smart cycle ${PARAMS.smart ? "on" : "off"}</p>
    <h2>Verdict</h2><p>${LIMITS.length ? "✗ " + LIMITS.map(svgEsc).join(" · ") : "✓ All limits hold: reach, carton overlaps, clearance ≥ 10 mm, interlocks, motor peak / RMS / speed, vacuum safety factor."}</p>
    <h2>Key data</h2><dl>${reportRows().map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("")}</dl>
    <div class="charts"><figure>${svg("chart-tq")}<figcaption>Motor torque over the run, both axes, with S1 and peak limits.</figcaption></figure>
    <figure>${svg("chart-ns")}<figcaption>Operating points on the motor envelope (S1 simplified between published points).</figcaption></figure>
    <figure>${svg("ws")}<figcaption>Working range in the robot plane and the flange path.</figcaption></figure></div>
    <h2>I/O list</h2>${$("io-list").innerHTML}
    <h2>Method and sources</h2><ul>
      <li>Kinematics: joint centres fitted on the bearing bores of the vendor STEP 3D-model_A_00913 (Autonox); range from the connecting-dimensions drawing 087.</li>
      <li>Dynamics: rigid bodies with mass, centre of mass and planar inertia from the STEP (signed-volume integrals; aluminium 2700, carbon 1600 kg/m³); torques by virtual work; checked against dV/dθ in the test suite. Carbon tubes are modelled with thick walls in the STEP: masses are conservative.</li>
      <li>Motor: ${MOTOR.name} as recommended on the Autonox page; M0 ${MOTOR.M0} Nm, Mmax ${MOTOR.Mmax} Nm, Mn ${MOTOR.Mn} Nm at ${MOTOR.nN} rpm, nmax ${MOTOR.nMax} rpm, J ${MOTOR.J * 1e4} kg·cm² (Beckhoff Infosys, AM805x, winding N, 400 V). The gearbox ratio is not published: i = ${PARAMS.ratio} is an assumption.</li>
      <li>Vacuum: ${VAC.dp / 1000} kPa effective on corrugated board, ${VAC.sealed * 100} % of the ${TOOL.padX * 1000} × ${TOOL.padY * 1000} mm pad sealed, μ = ${VAC.mu}, required safety factor ${VAC.sfReq}.</li>
      <li>Not modelled: joint friction, gearbox backlash and stiffness, cable forces, controller dynamics. Results are a sizing estimate, not a validated machine.</li></ul>`;
}
$("print-btn").addEventListener("click", () => { buildPrintReport(); window.print(); });
