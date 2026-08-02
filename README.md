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

For a regional artifact that will replace columns in an existing full cache,
also pass `--global-bounds minX minY minZ maxX maxY maxZ`. This keeps the
regional bake in the full cache's exact origin, tile-coordinate system, and
capacity even though `--bounds` limits the work. The global extent and every
profile/voxel/agent setting must match the destination cache exactly.

## Deterministic regional TileCache overlays

`tilecache-overlay` replaces complete `(tx,ty)` columns in a full cache. It
never combines individual layers from the old and new versions of one column.
The regional source must include at least a one-tile geometry halo on every
side of the smaller replacement rectangle; halo layers are validated but are
not copied into the result.

```sh
./build/tilecache-overlay \
  --base-dir ./full-cache \
  --overlay-dir ./regional-cache \
  --output-dir ./merged-cache \
  --replace-coverage 148 112 152 118 \
  --overlay-coverage 146 110 153 119
```

Both rectangles use half-open tile coordinates. Inputs are rejected on part
gaps, malformed/trailing payloads, duplicate or non-contiguous layers,
coordinate/bounds inconsistencies, or any origin, capacity, voxel, walkability,
or simplification mismatch. Output records are sorted canonically, assigned
deterministic references, split deterministically, re-read for verification,
and accompanied by a SHA-256 report. The output directory must not already
exist, which prevents a failed run from mixing new parts with stale ones.

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

Strict semantic collision is applied at the source triangle's voxel footprint,
not its axis-aligned bounding box. Component-specific walkable faces may
override a coplanar coarse `nav_obstacle_static` face only when both belong to
the same OBJ object; a separate prop object at the same location still blocks
navigation. Explicit semantic obstacles also bypass Recast's legacy
low-hanging-obstacle promotion, which can otherwise turn a tagged blocker back
into a walkable span.

Canonical semantic regions are authored topology, so `--region-min` pruning is
disabled for semantic input. Short thresholds and individual stair components
must survive once classified; decoration/noise rejection belongs in the
classifier. `--region-min` continues to apply to
`--legacy-object-fallback` input. Set `H1EMU_NAV_DEBUG_SEMANTIC_STAGES=1` to log
compact-heightfield area histograms before carving, after carving, and after
erosion while diagnosing a regional bake.

Every run writes `<output>.semantics.json` with the semantic contract,
deterministic material histogram, taxonomy, warnings, normalized input basename
and content identity, dynamic-door acknowledgement, and legacy-mode state. It
never records the caller-spelled input path. Use `--semantic-report <path>` to
choose another path.
`--validate-semantics-only --require-all-semantics` validates a fixture without
building any tiles. `--verify-baked-semantics` additionally inspects direct and
materialized TileCache polygons after a bake, including static-obstacle
footprint probes. Obstacle checks are multi-storey aware: they reject a
walkable span at the obstacle layer or below it without enough headroom, but do
not reject an independent floor above the blocker. Individual obstacle faces
smaller than one quarter of a raster cell are not required to own a stable
probe; larger source blockers and their eroded agent-radius clearance remain
verified. With `--bounds`, retained area IDs are required only for
canonical source triangles that have non-zero X/Z overlap with the requested
bounds and pass the configured Recast slope test. Materials found only in the
rest of a source chunk cannot make a regional verification fail; in-bounds
walkable source coverage still must survive in both outputs with the canonical
flags.
