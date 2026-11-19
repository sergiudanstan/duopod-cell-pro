# Mass properties of each DuoPod part from the vendor GLB (local only, _cad/ is git-ignored).
# Usage: uv run --python 3.13 --with trimesh --with numpy --with scipy python -I tools/cad/mass.py _cad/duopod.glb
# Output in the robot plane frame (x_r = y_cad, z_r = z_cad), SI units.
import sys, json
import numpy as np, trimesh
from collections import defaultdict

DENS = {"alu": 2700.0, "carbon": 1600.0, "steel": 7850.0}
s = trimesh.load(sys.argv[1])
groups = defaultdict(list)
for name in s.graph.nodes_geometry:
    T, g = s.graph[name]
    m = s.geometry[g].copy(); m.apply_transform(T)
    key = name.rsplit("_", 1)[0]
    mat = m.visual.material if hasattr(m.visual, "material") else None
    col = np.array(getattr(mat, "baseColorFactor", None) if mat is not None and getattr(mat, "baseColorFactor", None) is not None else [150, 150, 150, 255], float)
    if col.max() > 1.5: col = col / 255.0
    groups[key].append((m, col))
out = {}
for key, items in sorted(groups.items()):
    mass = 0.0; mom = np.zeros(3); details = []; J0 = 0.0   # J0: second moment about the CAD x axis through the origin (plane normal)
    for m, col in items:
        lum = col[:3].mean()
        dens = DENS["carbon"] if lum < 0.06 else DENS["alu"]
        mm = m.copy(); mm.merge_vertices(digits_vertex=6)
        wt = mm.is_watertight
        # signed (divergence-theorem) volume works on B-rep tessellations with hairline cracks; the hull is only a bound
        tri = mm.triangles; cr = np.cross(tri[:, 1], tri[:, 2]); v6 = np.einsum("ij,ij->i", tri[:, 0], cr)
        vol = v6.sum() / 6.0
        cen = (v6[:, None] * tri.sum(1)).sum(0) / (4.0 * v6.sum()) if abs(v6.sum()) > 1e-12 else mm.centroid
        hull = mm.convex_hull.volume
        if vol <= 0 or vol > hull * 1.001: vol, cen = hull, mm.convex_hull.center_mass   # orientation broken: fall back to the hull (upper bound)
        # second moment about the x axis through the origin: integral of (y^2 + z^2) dm over signed tetrahedra (Tonon 2004)
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        def s2(k): return (a[:, k]**2 + b[:, k]**2 + c[:, k]**2 + a[:, k]*b[:, k] + a[:, k]*c[:, k] + b[:, k]*c[:, k])
        jx = (v6 * (s2(1) + s2(2))).sum() / 60.0
        if vol == hull: jx = mm.convex_hull.mass_properties["inertia"][0][0] + hull * (cen[1]**2 + cen[2]**2) if False else hull * (cen[1]**2 + cen[2]**2)
        J0 += jx * dens
        mass += vol * dens; mom += vol * dens * cen
        details.append({"tris": len(mm.faces), "watertight": bool(wt), "vol_cm3": round(vol * 1e6, 1), "hull_cm3": round(hull * 1e6, 1), "dens": dens})
    com = mom / mass if mass else mom
    Jc = J0 - mass * (com[1]**2 + com[2]**2)   # about the axis through the centre of mass
    out[key] = {"mass_kg": round(mass, 4), "com_xz": [round(com[1], 5), round(com[2], 5)], "J_com": round(Jc, 5), "parts": details}
print(json.dumps(out, indent=1))
