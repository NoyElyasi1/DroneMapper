# DroneMapper

## Contributors
- **Roni Hagai** — 324207257
- **Noy Elyasi** — 315659201

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
