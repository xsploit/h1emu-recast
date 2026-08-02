# h1emu-recast

Builds a navmesh (and tilecache) from a world `.obj` mesh using
[recastnavigation](https://github.com/recastnavigation/recastnavigation).

## Build

```sh
git submodule update
cmake -S ./ -B ./build -DCMAKE_BUILD_TYPE=Release
cmake --build ./build -j$(nproc)
```

On Windows, run CMake from an x64 Native Tools prompt:

```bat
cmake -S . -B build-msvc -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-msvc --parallel
```

## Run

```sh
./build/navmesh-builder ./world-semantic.obj ./out/navmesh.bin --profile human \
  --dynamic-door-obstacles
```

The builder takes the path to a world `.obj` file and outputs the generated
navmesh/tilecache data.

`--profile human` selects the fine streaming configuration used by h1emu:
0.2-meter horizontal cells, 0.1-meter vertical cells, 0.2-meter agent radius,
1.3-meter climb, 45-degree slope, and 128-voxel tiles. The reduced erosion is
required to retain the original game's narrow interior doorways.

Use `--bounds minX minZ maxX maxZ` for repeatable regional validation before a
full-map bake. Individual settings can be overridden with `--cell-size`,
`--cell-height`, `--agent-height`, `--agent-radius`, `--agent-climb`,
`--agent-slope`, `--tile-size`, `--region-min`, and `--region-merge`.

## Semantic geometry contract

Semantic OBJ input assigns every face exactly one canonical `usemtl` value:

| Material | Area | Polygon flags | Bake action |
| --- | ---: | --- | --- |
| `nav_terrain` | 1 | `WALK` | rasterize |
| `nav_road` | 2 | `WALK` | rasterize |
| `nav_floor_exterior` | 3 | `WALK` | rasterize |
| `nav_floor_interior` | 4 | `WALK \| INDOOR` | rasterize |
| `nav_stair` | 5 | `WALK \| TRANSITION` | rasterize |
| `nav_ramp` | 6 | `WALK \| TRANSITION` | rasterize |
| `nav_threshold` | 7 | `WALK \| TRANSITION \| DOOR` | rasterize |
| `nav_obstacle_static` | 0 | none | rasterize non-walkable |
| `nav_door_panel_dynamic` | 0 | none | exclude for runtime door obstacle |
| `nav_exclude` | 0 | none | exclude |
| `nav_unknown` | 0 | none | rasterize non-walkable and warn |

Flag values are `WALK=0x01`, `INDOOR=0x02`, `TRANSITION=0x04`, and
`DOOR=0x08`. Unknown `nav_*` materials are fatal. Untagged faces and ordinary
materials are fatal in semantic input, preventing silent taxonomy drift.

`nav_door_panel_dynamic` intentionally removes the baked door panel so the
server can own the closed/open blocker through TileCache. A strict semantic
bake containing this material fails unless `--dynamic-door-obstacles` is
present. The acknowledgement is recorded in semantic provenance and must be
paired with deployment that enables and validates runtime door obstacles.

Pre-semantic OBJ files are supported only with the explicit
`--legacy-object-fallback` option. This enables the historical object-name
rules without allowing them to override canonical semantic faces.

Every run writes `<output>.semantics.json` with the semantic contract,
deterministic material histogram, taxonomy, warnings, normalized input basename
and content identity, dynamic-door acknowledgement, and legacy-mode state. It
never records the caller-spelled input path. Use `--semantic-report <path>` to
choose another path.
`--validate-semantics-only --require-all-semantics` validates a fixture without
building any tiles. `--verify-baked-semantics` additionally inspects direct and
materialized TileCache polygons after a bake, including static-obstacle
footprint probes. With `--bounds`, retained area IDs are required only for
canonical source triangles that have non-zero X/Z overlap with the requested
bounds and pass the configured Recast slope test. Materials found only in the
rest of a source chunk cannot make a regional verification fail; in-bounds
walkable source coverage still must survive in both outputs with the canonical
flags.
