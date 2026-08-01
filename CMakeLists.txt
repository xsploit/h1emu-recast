cmake_minimum_required(VERSION 3.16)
project(navmesh-builder C CXX)

include(CTest)

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

if(BUILD_TESTING)
    add_test(NAME semantic-contract
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/semantic_contract.obj
            ${CMAKE_BINARY_DIR}/semantic-contract.bin
            --validate-semantics-only
            --require-all-semantics
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-contract.json
    )
    add_test(NAME semantic-invalid-material
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/invalid_semantic.obj
            ${CMAKE_BINARY_DIR}/semantic-invalid.bin
            --validate-semantics-only
    )
    set_tests_properties(semantic-invalid-material PROPERTIES WILL_FAIL TRUE)

    add_test(NAME semantic-ordinary-material-requires-legacy
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/ordinary_material.obj
            ${CMAKE_BINARY_DIR}/semantic-ordinary-rejected.bin
            --validate-semantics-only
    )
    set_tests_properties(semantic-ordinary-material-requires-legacy
        PROPERTIES WILL_FAIL TRUE)

    add_test(NAME semantic-ordinary-material-explicit-legacy
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/ordinary_material.obj
            ${CMAKE_BINARY_DIR}/semantic-ordinary.bin
            --legacy-object-fallback
            --validate-semantics-only
    )

    add_test(NAME semantic-legacy-requires-opt-in
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/legacy_object.obj
            ${CMAKE_BINARY_DIR}/semantic-legacy-rejected.bin
            --validate-semantics-only
    )
    set_tests_properties(semantic-legacy-requires-opt-in PROPERTIES WILL_FAIL TRUE)

    add_test(NAME semantic-legacy-explicit
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/legacy_object.obj
            ${CMAKE_BINARY_DIR}/semantic-legacy.bin
            --legacy-object-fallback
            --validate-semantics-only
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-legacy.json
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
