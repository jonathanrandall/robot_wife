#!/usr/bin/env python3
"""Re-run to regenerate ../visual/*.stl after a STEP file changes.

Requires cadquery (pip install cadquery) - the user's ~/venvs/jazzy venv has it.
"""
import os
import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
VISUAL_DIR = os.path.join(HERE, "..", "visual")

PARTS = {
    "wheel_120":     "wheel_120.step",
    "foot_360":      "foot_360.step",
    "cross_bar_144": "cross_bar_144.step",
    "leg_528":       "leg_528.step",
    "grid_88_232":   "grid_88_232.step",
    "body_432":      "body_432.step",
}

if __name__ == "__main__":
    os.makedirs(VISUAL_DIR, exist_ok=True)
    for name, filename in PARTS.items():
        step_path = os.path.join(HERE, filename)
        result = cq.importers.importStep(step_path)
        bb = result.val().BoundingBox()
        print(f"{name}: xlen={bb.xlen:.2f} ylen={bb.ylen:.2f} zlen={bb.zlen:.2f} "
              f"center=({bb.center.x:.2f}, {bb.center.y:.2f}, {bb.center.z:.2f})")
        out_path = os.path.join(VISUAL_DIR, f"{name}.stl")
        cq.exporters.export(result, out_path, tolerance=0.05, angularTolerance=0.2)
