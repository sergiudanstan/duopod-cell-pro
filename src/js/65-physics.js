// ---------------------------------------------------------------- MuJoCo physics replay: load the recorded run, switch the source of the cycle
// Data: physics/duopod_physics.json, written by duopod-cell-mujoco/tools/export_trajectory.py (real contacts, friction,
// adhesion gripper, closed-chain dynamics). ?physics=0 starts in the kinematic model.
let physData = null;
const simButtons = document.querySelectorAll(".sim-switch button");
function setSimSource(src) {
  const phys = src === "physics" && physData;
  PHYS = phys ? physData : null;
  simButtons.forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.src === (phys ? "physics" : "kinematic"))));
  document.querySelectorAll("#whatif input").forEach((i) => { i.disabled = false; });
  $("wi-opt").disabled = !!phys;
  $("sim-note").textContent = phys
    ? `Showing the recorded ${physData.meta.engine} run: contacts, friction, vacuum adhesion, closed-chain dynamics (1 kHz control). Dashed lines: torque measured in the physics. Moving a slider switches back to the kinematic model.`
    : "Kinematic model: event simulation with analytic dynamics. What-if sliders rebuild it live.";
  rebuild(); refreshModel();
  time = 0;
}
simButtons.forEach((b) => b.addEventListener("click", () => setSimSource(b.dataset.src)));
(async () => {
  try {
    const r = await fetch("physics/duopod_physics.json");
    if (!r.ok) throw new Error(r.status);
    const d = await r.json();
    d.n = d.t.length; d.fps = d.meta.fps;
    physData = d;
    simButtons.forEach((b) => { b.disabled = false; });
    if (new URLSearchParams(location.search).get("physics") !== "0") setSimSource("physics");
    window.dispatchEvent(new Event("physicsloaded"));
  } catch (err) {
    $("sim-note").textContent = "Physics replay not available (physics/duopod_physics.json missing): kinematic model only.";
    simButtons.forEach((b) => { if (b.dataset.src === "physics") b.disabled = true; });
  }
})();
