Assignment 2 — DroneMapper

OUTPUT FILE NAMING SCHEME
==========================

All output files are written under <output_path>/output_results/.

Per-run directory name format:
  <map_stem>_ms<max_steps>_d<drone_dims_cm>_fovc<lidar_fov_circles>/

Where:
  <map_stem>       = stem of the simulation's map_filename (e.g. "office" from "maps/office.npy")
  <max_steps>      = mission's max_steps value
  <drone_dims_cm>  = integer centimeter value of drone.dimensions
  <fovc>           = lidar's fov_circles count

Files inside each run directory:
  output_map.npy   — The drone's occupancy map as a NumPy int8 3D array
                     (shape = [szX, szY, szZ], origin at mission boundary min corner)
  errors.log       — Error log file (written and flushed immediately on each error)

Top-level output:
  simulation_output.yaml  — SimulationReport as YAML (scores, status, timestamps)

EXAMPLE
-------
For simulation map "maps/office.npy", mission max_steps=2400,
drone 30cm, lidar fov_circles=5:

  output_path/
    simulation_output.yaml
    output_results/
      office_ms2400_d30_fovc5/
        output_map.npy
        errors.log

RESOLUTION LOGIC
================
actual_resolution = gps_resolution_cm * output_mapping_resolution_factor

If output_mapping_resolution_factor < 1:
  → Use gps_resolution_cm, status = Ignored (logged as error)

If actual_resolution < map_resolution_cm / 2:
  → Use gps_resolution_cm, status = IgnoredTooSmall

Otherwise:
  → Use actual_resolution, status = Accepted
