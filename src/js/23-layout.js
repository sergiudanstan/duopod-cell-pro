// ================================================================ shared 3D layout (metres, z up, belt top z = 0). Read-only for the scene fragments.
const LAYOUT = {
  floor: -0.85,                                   // belt height 850 mm
  posts: { x: 0.80, y: 0.72 },                    // 80x80 portal posts at (±x, ±y)
  topZ: [ROBOT.mountZ + 0.030, ROBOT.mountZ + 0.110],   // 80x80 beams on top of the 30 mm mounting plate
  carriers: SUPPORT.carrierX,                      // two 80x80 carriers along y over the robot head (see SUPPORT in the logic)
  plate: { x: 2 * SUPPORT.plateX, y: 2 * SUPPORT.plateY, t: SUPPORT.plateT },   // mounting plate = head footprint (underside = mounting face)
  guardTop: ROBOT.mountZ + 0.110,
  opening: { x: 0.42, z: [-0.16, 0.24] },         // belt openings in the end walls
  stack: { x: 0.80, y: 0.72 },
  hmi: { x: -0.80, y: 0.30, z: 0.45 },             // on the outer face of the -x side, facing -x
};
