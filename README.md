# DroneMapper

## Contributors
- **Roni Hagai** — 324207257
- **Noy Elyasi** — 315659201

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Running

```bash
./drone_mapper [<input_output_files_path>]
```

If `<input_output_files_path>` is omitted, the current working directory is used.

## Input Files

The program expects three input files in the given directory:

### `drone_config.txt`
Key=value pairs defining the drone's physical capabilities:
- `width`, `length`, `height` — drone bounding box dimensions (cm)
- `max_rotate` — maximum rotation per command (degrees)
- `max_advance` — maximum horizontal movement per command (cm)
- `max_elevate` — maximum vertical movement per command (cm)
- `lidar_zmin` — minimum measurable distance (cm)
- `lidar_zmax` — maximum operational range (cm)
- `lidar_d` — circle spacing at Zmin (cm)
- `lidar_fovc` — number of beam circles (0 = central beam only)

Lines starting with `#` are comments.

### `mission_config.txt`
Key=value pairs defining the mapping mission:
- `min_x`, `max_x`, `min_y`, `max_y`, `min_height`, `max_height` — mapping boundaries (cm)
- `res_x`, `res_y`, `res_height` — resolution: number of decimal places (0 = 1 cm grid)
- `start_x`, `start_y`, `start_z` — optional starting position (cm)

Lines starting with `#` are comments.

### `map_input.txt`
Ground-truth building map. Each non-comment line is a comma-separated row:
```
x,y,z,status
```
- `x`, `y`, `z` — integer grid indices
- `status` — `1` = occupied (wall/floor/ceiling)

Lines starting with `#` are comments.

## Output File

### `output_map.txt`
Written to the same directory as the input files. Same CSV format as `map_input.txt`:
```
x,y,z,status
```
- `0` = empty (scanned, confirmed open)
- `1` = occupied (scanned, confirmed solid)
- `-1` = unmapped (not yet reached / unreachable)

## External Libraries

### mp-units (v2.4.0)
- **Repository:** https://github.com/mpusz/mp-units
- **Purpose:** Compile-time dimensional analysis for physical quantities (distances in centimetres, angles in degrees). Prevents unit-mismatch bugs at compile time.
- **Usage:** All physical quantities in the project (distances, angles, coordinates) are wrapped in mp-units types defined in `include/Types.hpp`. This ensures that, for example, a distance in centimetres cannot be accidentally added to an angle in degrees.
- **Integration:** Fetched automatically via CMake `FetchContent` — no manual installation required.
