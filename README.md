# DuoPod Transfer Cell Pro

**Live:** https://sergiudanstan.github.io/duopod-cell-pro/

This is the pro version of [duopod-cell](https://sergiudanstan.github.io/duopod-cell/). A ceiling-mounted 2-axis parallel robot moves 3 kg cartons from an infeed belt to the belt beside it. The robot is modelled on the Autonox DuoPod RVS2-1000-3kg, A_00913.

The vendor CAD is never published. The public page uses a detailed procedural model built on the measured dimensions.

## What is new in Pro

### Engineering
- **Rigid-body dynamics.** Mass, centre of mass and planar inertia of every part come from the vendor STEP (signed-volume integrals). Torques are computed by virtual work, and the test suite checks them against dV/dθ.
- **Motor sizing.** The motor is the Beckhoff AM8053-0N21 that Autonox recommends:
  - motor-side torque τ/(i·η) + J·i·θ̈;
  - peak, RMS and speed are each checked against Mmax, the S1 line and nmax.
  - The gearbox ratio is not published, so it is an assumption you can change in What-if.
- **Vacuum safety factor** on corrugated board.
- **Time-optimal motion scale.** Bisection finds the fastest scale that keeps every limit, and names the limit that binds.
- **Smart cycle.** The robot returns straight to the next carton, but only when the photo-eye has confirmed it before the descent starts.
- **Place-zone interlock on belt B.**

### Interactive
- Live what-if sliders: belt speeds, payload, travel height, gearbox ratio, time scale and motor limit.
- Torque–time and torque–speed charts.
- HMI pages on the 3D panel.
- I/O list and a Structured Text program generated from the simulation constants.
- Printable report (browser "Save as PDF").

### Look
- Detailed procedural DuoPod.
- Real 80×80 slot-8 profiles.
- Acrylic guards with a door and safety switch, plus light curtains at the belt openings.
- Electrical cabinet, air preparation unit and cable tray.
- Conveyors with drums, return run, bearings, adjustable guides and geared motors.
- Pneumatic escapement with a valve terminal and PU tubes.
- Labelled cartons.

## Checks

```sh
node tools/build.mjs           # concatenate src/ into index.html
node tests/run.mjs             # kinematics, range, cartons, clearances, dynamics, motor, vacuum, smart cycle, optimizer, what-if sweep
node tests/run.mjs --perturb   # escapement released on vacuum OK: must fail (exit 1)
node tools/shots.mjs           # stills, console errors, fps, CAD-vs-logic check
node tools/record.mjs          # ~/Downloads/duopod_cell_pro/duopod_cell_pro.mp4
uv run --python 3.13 --with trimesh --with numpy --with scipy python -I tools/cad/mass.py _cad/duopod.glb   # mass properties (local CAD)
```

Results are a sizing estimate, not a validated machine. The model leaves out joint friction, backlash, gearbox stiffness and cable forces.

Autonox, DuoPod and Beckhoff are trademarks of their owners.
