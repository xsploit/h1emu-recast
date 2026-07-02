# h1emu-recast

Builds a navmesh (and tilecache) from a world `.obj` mesh using
[recastnavigation](https://github.com/recastnavigation/recastnavigation).

## Build

```sh
git submodule update
cmake -S ./ -B ./build -DCMAKE_BUILD_TYPE=Release
cmake --build ./build -j$(nproc)
```

## Run

```sh
./build/navmesh-builder ./world.obj
```

The builder takes the path to a world `.obj` file and outputs the generated
navmesh/tilecache data.
