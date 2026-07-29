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
./build/navmesh-builder ./world.obj ./out/navmesh.bin --profile human
```

The builder takes the path to a world `.obj` file and outputs the generated
navmesh/tilecache data.

`--profile human` selects the fine streaming configuration used by h1emu:
0.2-meter horizontal cells, 0.1-meter vertical cells, 0.2-meter agent radius,
0.5-meter climb, 45-degree slope, and 128-voxel tiles. The reduced erosion is
required to retain the original game's narrow interior doorways.

Use `--bounds minX minZ maxX maxZ` for repeatable regional validation before a
full-map bake. Individual settings can be overridden with `--cell-size`,
`--cell-height`, `--agent-height`, `--agent-radius`, `--agent-climb`,
`--agent-slope`, `--tile-size`, `--region-min`, and `--region-merge`.
