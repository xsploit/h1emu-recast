cmake_minimum_required(VERSION 3.16)
project(navmesh-builder C CXX)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(RECAST_DIR ${CMAKE_SOURCE_DIR}/recastnavigation)

file(GLOB RECAST_SOURCES ${RECAST_DIR}/Recast/Source/*.cpp)
file(GLOB DETOUR_SOURCES ${RECAST_DIR}/Detour/Source/*.cpp)
file(GLOB DETOUR_TILECACHE_SOURCES ${RECAST_DIR}/DetourTileCache/Source/*.cpp)

add_executable(navmesh-builder
    main.cpp
    ${RECAST_SOURCES}
    ${DETOUR_SOURCES}
    ${DETOUR_TILECACHE_SOURCES}
    ${RECAST_DIR}/RecastDemo/Contrib/fastlz/fastlz.c
)

target_include_directories(navmesh-builder PRIVATE
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include
    ${RECAST_DIR}/DetourTileCache/Include
    ${RECAST_DIR}/RecastDemo/Include
    ${RECAST_DIR}/RecastDemo/Contrib/fastlz
)

target_compile_definitions(navmesh-builder PRIVATE
    # Dense multi-storey interiors can exceed Recast's conservative default
    # of 63 overlapping heightfield layers. Layer ids are stored as bytes, so
    # 255 is the maximum supported value.
    RC_MAX_LAYERS_DEF=255
)

if(MSVC)
    target_compile_options(navmesh-builder PRIVATE /W4 /O2 /fp:fast)
    target_compile_definitions(navmesh-builder PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(navmesh-builder PRIVATE
        -Wall -Wextra
        -march=native
        -ffast-math
    )
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(navmesh-builder PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(navmesh-builder PRIVATE USE_OPENMP)
    message(STATUS "OpenMP found — tile building will use parallel threads")
else()
    message(STATUS "OpenMP not found — building single-threaded")
endif()
