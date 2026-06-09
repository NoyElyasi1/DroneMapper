#!/usr/bin/env python3
"""
convert_map.py — Convert map_input.txt (CSV grid-index format) to map_input.npy

Usage:
    python convert_map.py <sample_dir>
    python convert_map.py sample1
    python convert_map.py         # converts all sample* directories

The script reads:
    <dir>/map_input.txt      (format: x,y,z,status  with integer grid indices)
    <dir>/mission_config.yaml

And writes:
    <dir>/map_input.npy      (3D int8 NumPy array, shape (nx, ny, nz))

Array index mapping:
    array[ix, iy, iz] = status
    where ix = x_grid_index - offset_x
          iy = y_grid_index - offset_y
          iz = z_grid_index - offset_z
    and offset_* = round(min_* / step)

Requires: numpy, pyyaml
    pip install numpy pyyaml
"""

import sys
import os
import glob
import math

import numpy as np
import yaml


def load_mission(yaml_path: str) -> dict:
    with open(yaml_path) as f:
        root = yaml.safe_load(f)
    mc = root.get("mission_config", {})
    bounds = mc.get("boundaries", {})

    gps_res = float(mc.get("gps_resolution_cm", 10.0))
    factor  = int(mc.get("mapping_resolution_factor", 1))
    step    = gps_res / max(factor, 1)

    xb = bounds.get("x_boundary",      {})
    yb = bounds.get("y_boundary",      {})
    hb = bounds.get("height_boundary", {})

    return {
        "min_x":      float(xb.get("min_cm",  -500)),
        "max_x":      float(xb.get("max_cm",   500)),
        "min_y":      float(yb.get("min_cm",  -500)),
        "max_y":      float(yb.get("max_cm",   500)),
        "min_height": float(hb.get("min_cm",     0)),
        "max_height": float(hb.get("max_cm",   300)),
        "step":       step,
    }


def convert_directory(sample_dir: str) -> None:
    txt_path  = os.path.join(sample_dir, "map_input.txt")
    yaml_path = os.path.join(sample_dir, "mission_config.yaml")
    npy_path  = os.path.join(sample_dir, "map_input.npy")

    if not os.path.isfile(txt_path):
        print(f"  [skip] {txt_path} not found")
        return
    if not os.path.isfile(yaml_path):
        print(f"  [skip] {yaml_path} not found")
        return

    mc   = load_mission(yaml_path)
    step = mc["step"]

    # Compute grid offsets and dimensions
    off_x = int(round(mc["min_x"]      / step))
    off_y = int(round(mc["min_y"]      / step))
    off_z = int(round(mc["min_height"] / step))

    nx = int(round((mc["max_x"]      - mc["min_x"])      / step)) + 1
    ny = int(round((mc["max_y"]      - mc["min_y"])      / step)) + 1
    nz = int(round((mc["max_height"] - mc["min_height"]) / step)) + 1

    grid = np.zeros((nx, ny, nz), dtype=np.int8)

    bad_lines = 0
    with open(txt_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 4:
                bad_lines += 1
                continue
            try:
                gx, gy, gz, status = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
            except ValueError:
                bad_lines += 1
                continue

            ix = gx - off_x
            iy = gy - off_y
            iz = gz - off_z
            if 0 <= ix < nx and 0 <= iy < ny and 0 <= iz < nz:
                grid[ix, iy, iz] = status

    np.save(npy_path, grid)

    if bad_lines:
        print(f"  [warn] {bad_lines} bad lines skipped in {txt_path}")
    print(f"  [ok]   {npy_path}  shape={grid.shape}  occupied={int((grid != 0).sum())}")


def main() -> None:
    if len(sys.argv) > 1:
        dirs = sys.argv[1:]
    else:
        dirs = sorted(glob.glob("sample*"))

    if not dirs:
        print("No sample directories found.")
        return

    for d in dirs:
        print(f"Converting {d}/...")
        convert_directory(d)


if __name__ == "__main__":
    main()
