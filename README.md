# DroneMapper — Exercise 2

## Contributors
- **Roni Hagai** — 324207257
- **Noy Elyasi** — 315659201

## External Libraries (Approved)
- **mp-units** — compile-time physical units (distances in cm, angles in degrees). Prevents unit-mismatch bugs.
- **yaml-cpp** — YAML configuration file parsing.
- **TinyNPY** — NumPy `.npy` file reading/writing for 3-D voxel maps.
- **GTest / GMock** — unit and component testing framework (test binary only).

## Build Requirements

Requires **g++-12** or later (gcc 11 has a known compiler bug with the `mp-units` dependency).
Requires vcpkg with `VCPKG_ROOT` set.

```bash
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset default
cmake --build build
```

This produces three binaries under `build/`:
| Binary | Purpose |
|---|---|
| `drone_mapper_simulation` | Main simulator — runs YAML-based composition |
| `maps_comparison` | Standalone map scorer |
| `drone_mapper_simulation_test` | GTest test suite (114 tests) |

## Running the Simulator

```bash
./build/drone_mapper_simulation [<composition_file>] [<output_path>]
```

- `composition_file` (optional): path to a `sim_compose.yaml` file.  
  Defaults to `simulation.yaml` in the current working directory.
- `output_path` (optional): directory for output files.  
  Defaults to the current working directory.

### Sample scenarios
Three self-contained scenarios are provided:

| Folder | Map | Scenario |
|---|---|---|
| `sample1/` | `scenario_small` (20×20×20 voxels) | Outdoor open-space flight |
| `sample2/` | `scenario_big` (30×30×30 voxels) | Confined interior room |
| `sample3/` | `scenario_house` (29×30×31 voxels, height_offset=150 cm) | Multi-floor building — lower floor |

Run any sample:
```bash
./build/drone_mapper_simulation sample1/sim_compose.yaml sample1
```

Original outputs (from our run) are preserved in `sampleX/original_output/`.

## Input File Format

Each composition file (`sim_compose.yaml`) references:

```
sim_compose.yaml              ← top-level composition
simulation.yaml               ← map file, initial drone position, map offset
mission.yaml                  ← max_steps, exploration boundaries, GPS resolution
drone.yaml                    ← drone dimensions, max_rotate/advance/elevate
lidar.yaml                    ← z_min, z_max, d (circle spacing), fov_circles
map/<name>.npy                ← 3-D NumPy int8 array (0=Empty, 1=Occupied)
```

### simulation.yaml fields
| Field | Meaning |
|---|---|
| `map_filename` | Path to `.npy` map (relative to this yaml's directory) |
| `map_resolution_cm` | Voxel size in cm |
| `initial_drone_position` | `x_cm`, `y_cm`, `height_cm` — in **map-local** coordinates |
| `initial_angle_deg` | Starting heading (0=east, 90=south) |
| `map_axes_offset` | `x_offset`, `y_offset`, `height_offset` — shifts map origin to world coords |

> **Note:** `initial_drone_position` and mission `boundaries` are in map-local coordinates. The `map_axes_offset` is added internally to convert them to world coordinates.

### mission.yaml fields
| Field | Meaning |
|---|---|
| `max_steps` | Maximum number of drone steps |
| `boundaries` | Exploration volume in map-local cm (`x_boundary`, `y_boundary`, `height_boundary`) |
| `gps_resolution_cm` | GPS measurement precision |
| `output_mapping_resolution_factor` | Output voxel size multiplier (≥1, default 1) |

## Output File Format

All output is written to `<output_path>/`:

```
<output_path>/
  simulation_output.yaml       ← hierarchical score report
  output_results/
    errors.log                 ← error log (empty on success)
    <map>_ms<N>_d<R>_fovc<F>/
      output_map.npy           ← drone's occupancy map (NumPy int8 3-D array)
```

### output_map.npy
- Shape: `[szX, szY, szZ]` derived from mission boundaries and resolution
- Values: `0` = Empty (explored, no obstacle), `1` = Occupied (wall/obstacle detected), `-1` = Unmapped (not visited)

### simulation_output.yaml
Reports per-run score (0–100), status (`completed` / `max_steps` / `error`), and step count.

## Running the Tests

```bash
./build/drone_mapper_simulation_test                         # all 114 tests
./build/drone_mapper_simulation_test --gtest_filter=Integration.*
./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/drone_mapper_simulation_test --gtest_filter=MissionsControl.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*
```


## Build Requirements

Requires **g++-12** or later (gcc 11 has a known compiler bug with the `mp-units` dependency).
Requires vcpkg with `VCPKG_ROOT` set.

```bash
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset default
cmake --build build
```

## Running the Program

```bash
./drone_mapper_simulation [composition_file] [output_path]
```

- `composition_file` (optional): path to simulation composition YAML.
  - Missing → uses `simulation.yaml` in the current working directory.
  - Filename only → looked up in the current working directory.
  - Relative path → resolved under the current working directory.
  - Absolute path → used as-is.
- `output_path` (optional): where to write output files. Defaults to current working directory.

## Running the Tests

```bash
./drone_mapper_simulation_test                              # all tests
./drone_mapper_simulation_test --gtest_filter=Integration.*
./drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./drone_mapper_simulation_test --gtest_filter=SimulationRun.*
./drone_mapper_simulation_test --gtest_filter=MissionControl.*
./drone_mapper_simulation_test --gtest_filter=DroneControl.*
./drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./drone_mapper_simulation_test --gtest_filter=MockLidar.*
./drone_mapper_simulation_test --gtest_filter=MapsComparison.*
```

## Output File Naming Scheme

All output files are written under `<output_path>/output_results/`.

Per-run directory name:
```
<map_stem>_ms<max_steps>_d<drone_radius_cm>_fovc<lidar_fov_circles>/
```

- `map_stem` — stem of the simulation's map filename (e.g. `office` from `maps/office.npy`)
- `max_steps` — mission's max_steps value
- `drone_radius_cm` — integer cm value of drone radius
- `lidar_fov_circles` — lidar's fov_circles count

Files inside each run directory:
- `output_map.npy` — drone's occupancy map as a NumPy int8 3D array (shape `[szX, szY, szZ]`, origin at mission boundary min corner)

Top-level output:
- `simulation_output.yaml` — hierarchical scores and status
- `output_results/errors.log` — all errors logged immediately on occurrence

### Example layout
```
output_path/
  simulation_output.yaml
  output_results/
    errors.log
    office_ms2400_d30_fovc5/
      output_map.npy
```

## simulation_output.yaml Format

```yaml
score_report:
  composition_file: <path>
  generated_at_utc: <ISO 8601 timestamp>
  metric: "output_map_accuracy"
  score_range: { min: 0, max: 100 }
  error_score: -1
  summary:
    total_runs: <N>
    scored_runs: <N>
    error_runs: <N>
    average_score: <float>
    min_score: <float>
    max_score: <float>
  simulations:
    - simulation_config: <path>
      missions:
        - mission_config: <path>
          resolution_cm: <int>
          resolution_request_status: ACCEPTED | IGNORED | IGNORED TOO SMALL
          runs:
            - drone_config: <path>
              lidar_config: <path>
              status: completed | max_steps | error
              steps: <int>
              score: <float>          # -1 on error
              output_map_file: <path>
              error_ref:              # only on error
                code: <string>
```

## Resolution Logic

```
actual_resolution = gps_resolution_cm * output_mapping_resolution_factor
```
- Factor < 1 → use `gps_resolution_cm`, status = `IGNORED`
- `actual_resolution` < `map_resolution_cm / 2` → use `gps_resolution_cm`, status = `IGNORED TOO SMALL`
- Otherwise → use `actual_resolution`, status = `ACCEPTED`

## Error Handling

- All errors logged to `output_results/errors.log` immediately when they occur.
- A failed run gets `score: -1`; the simulation continues with remaining runs.
- A group that cannot start (e.g. bad map file) has all its runs set to `score: -1`.
