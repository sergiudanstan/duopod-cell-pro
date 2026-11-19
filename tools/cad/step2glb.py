# Converts the Autonox DuoPod STEP into a GLB for the local-only CAD mode (output stays in _cad/, which is git-ignored).
# Usage: uv run --python 3.13 --with cascadio==0.1.1 python tools/cad/step2glb.py _cad/src/3D-model_A_00913.stp _cad/duopod.glb
import sys
import cascadio

src, dst = sys.argv[1], sys.argv[2]
n = cascadio.step_to_glb(src, dst, tol_linear=0.1, tol_angular=0.35, tol_relative=False,
                         merge_primitives=True, use_parallel=True, include_materials=True)
print("step_to_glb:", src, "->", dst, "result", n)
