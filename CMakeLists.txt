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

add_executable(tilecache-overlay tilecache_overlay.cpp)

target_include_directories(tilecache-overlay PRIVATE
    ${RECAST_DIR}/Detour/Include
    ${RECAST_DIR}/DetourTileCache/Include
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
    target_compile_options(tilecache-overlay PRIVATE /W4 /O2)
    target_compile_definitions(tilecache-overlay PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(navmesh-builder PRIVATE
        -Wall -Wextra
        -march=native
        -ffast-math
    )
    target_compile_options(tilecache-overlay PRIVATE -Wall -Wextra)
endif()

if(BUILD_TESTING)
    add_test(NAME tilecache-overlay-self-test
        COMMAND tilecache-overlay --self-test)

    add_test(NAME semantic-contract
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/semantic_contract.obj
            ${CMAKE_BINARY_DIR}/semantic-contract.bin
            --validate-semantics-only
            --require-all-semantics
            --dynamic-door-obstacles
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-contract.json
    )
    add_test(NAME semantic-dynamic-door-requires-ack
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/semantic_contract.obj
            ${CMAKE_BINARY_DIR}/semantic-door-rejected.bin
            --validate-semantics-only
    )
    set_tests_properties(semantic-dynamic-door-requires-ack
        PROPERTIES WILL_FAIL TRUE)
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

    add_test(NAME semantic-bounded-bake
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/bake_semantics.obj
            ${CMAKE_BINARY_DIR}/semantic-bake/output.bin
            --profile human
            --region-min 1
            --region-merge 1
            --dynamic-door-obstacles
            --verify-baked-semantics
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-bake/semantics.json
    )

    add_test(NAME semantic-bounded-source-coverage
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/semantic-bounded-coverage
            -P ${CMAKE_SOURCE_DIR}/tests/verify_bounded_semantic_coverage.cmake
    )

    add_test(NAME semantic-slanted-obstacle-locality
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/semantic_slanted_obstacle.obj
            ${CMAKE_BINARY_DIR}/semantic-slanted-obstacle/output.bin
            --profile human
            --agent-climb 0.5
            --region-min 1
            --region-merge 1
            --dynamic-door-obstacles
            --verify-baked-semantics
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-slanted-obstacle/semantics.json
    )

    add_test(NAME semantic-small-threshold-retained
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/semantic_small_threshold.obj
            ${CMAKE_BINARY_DIR}/semantic-small-threshold/output.bin
            --profile human
            --agent-radius 0
            --dynamic-door-obstacles
            --verify-baked-semantics
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-small-threshold/semantics.json
    )

    add_test(NAME semantic-same-object-precedence
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/semantic-same-object-precedence
            -P ${CMAKE_SOURCE_DIR}/tests/verify_semantic_same_object_precedence.cmake
    )

    add_test(NAME semantic-report-path-independent
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/semantic-report-paths
            -P ${CMAKE_SOURCE_DIR}/tests/verify_path_independent_report.cmake
    )

    add_test(NAME global-bounds-overlay-contract
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DOVERLAY=$<TARGET_FILE:tilecache-overlay>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/global-bounds-overlay
            -P ${CMAKE_SOURCE_DIR}/tests/verify_global_bounds_overlay.cmake
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
