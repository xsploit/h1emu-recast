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

add_library(forgelight-nav-source STATIC
    forgelight_nav_source.cpp
    forgelight_heightmap_loader.cpp
    forgelight_instance_index.cpp
    forgelight_terrain_lattice.cpp
    forgelight_geometry_source.cpp
    navigation_transitions.cpp
)
target_include_directories(forgelight-nav-source PUBLIC ${CMAKE_SOURCE_DIR})
# nav_semantic.h (shared with main.cpp) only needs Recast.h for the
# RC_NULL_AREA constant -- header-only, no Recast .cpp sources need linking
# into this library.
target_include_directories(forgelight-nav-source PRIVATE
    ${RECAST_DIR}/Recast/Include)

add_executable(forgelight-nav-source-inspect
    forgelight_nav_source_inspect.cpp)
target_link_libraries(forgelight-nav-source-inspect PRIVATE
    forgelight-nav-source)

add_executable(navmesh-builder
    main.cpp
    compact_component_filter.cpp
    threshold_portal_geometry.cpp
    triangle_geometry.cpp
    ${RECAST_SOURCES}
    ${DETOUR_SOURCES}
    ${DETOUR_TILECACHE_SOURCES}
    ${RECAST_DIR}/RecastDemo/Contrib/fastlz/fastlz.c
)

target_link_libraries(navmesh-builder PRIVATE forgelight-nav-source)

target_include_directories(navmesh-builder PRIVATE
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include
    ${RECAST_DIR}/DetourTileCache/Include
    ${RECAST_DIR}/RecastDemo/Include
    ${RECAST_DIR}/RecastDemo/Contrib/fastlz
)

add_executable(tilecache-overlay
    tilecache_overlay.cpp
    ${DETOUR_SOURCES}
    ${DETOUR_TILECACHE_SOURCES}
    ${RECAST_DIR}/RecastDemo/Contrib/fastlz/fastlz.c)

target_link_libraries(tilecache-overlay PRIVATE forgelight-nav-source)

target_include_directories(tilecache-overlay PRIVATE
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include
    ${RECAST_DIR}/DetourTileCache/Include
    ${RECAST_DIR}/RecastDemo/Contrib/fastlz
)

target_compile_definitions(navmesh-builder PRIVATE
    # Dense multi-storey interiors can exceed Recast's conservative default
    # of 63 overlapping heightfield layers. Layer ids are stored as bytes, so
    # 255 is the maximum supported value.
    RC_MAX_LAYERS_DEF=255
)

if(MSVC)
    target_compile_options(forgelight-nav-source PRIVATE /W4 /O2)
    target_compile_definitions(forgelight-nav-source PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(forgelight-nav-source-inspect PRIVATE /W4 /O2)
    target_compile_options(navmesh-builder PRIVATE /W4 /O2 /fp:fast)
    target_compile_definitions(navmesh-builder PRIVATE _CRT_SECURE_NO_WARNINGS)
    # The histogram reconstructs TileCache polygons and must use the same
    # floating-point semantics as the baker it is auditing.
    target_compile_options(tilecache-overlay PRIVATE /W4 /O2 /fp:fast)
    target_compile_definitions(tilecache-overlay PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(forgelight-nav-source PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(forgelight-nav-source-inspect PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(navmesh-builder PRIVATE
        -Wall -Wextra
        -march=native
        -ffast-math
    )
    target_compile_options(tilecache-overlay PRIVATE
        -Wall -Wextra
        -march=native
        -ffast-math
    )
endif()

if(BUILD_TESTING)
    add_executable(compact-component-filter-test
        tests/compact_component_filter_test.cpp
        compact_component_filter.cpp
        ${RECAST_SOURCES})
    target_include_directories(compact-component-filter-test PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${RECAST_DIR}/Recast/Include)
    if(MSVC)
        target_compile_options(compact-component-filter-test PRIVATE /W4 /O2)
    else()
        target_compile_options(compact-component-filter-test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME compact-component-filter-contract
        COMMAND compact-component-filter-test)

    add_executable(threshold-portal-geometry-test
        tests/threshold_portal_geometry_test.cpp
        threshold_portal_geometry.cpp)
    target_include_directories(threshold-portal-geometry-test PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${RECAST_DIR}/Recast/Include)
    if(MSVC)
        target_compile_options(threshold-portal-geometry-test PRIVATE /W4 /O2)
    else()
        target_compile_options(threshold-portal-geometry-test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME threshold-portal-geometry-contract
        COMMAND threshold-portal-geometry-test)

    add_executable(triangle-geometry-test
        tests/triangle_geometry_test.cpp
        triangle_geometry.cpp)
    target_include_directories(triangle-geometry-test PRIVATE
        ${CMAKE_SOURCE_DIR})
    if(MSVC)
        target_compile_options(triangle-geometry-test PRIVATE /W4 /O2)
    else()
        target_compile_options(triangle-geometry-test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME triangle-geometry-contract
        COMMAND triangle-geometry-test)

    add_executable(forgelight-nav-source-test
        tests/forgelight_nav_source_test.cpp)
    target_link_libraries(forgelight-nav-source-test PRIVATE
        forgelight-nav-source)
    if(MSVC)
        target_compile_options(forgelight-nav-source-test PRIVATE /W4 /O2)
    else()
        target_compile_options(forgelight-nav-source-test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME forgelight-nav-source-contract
        COMMAND forgelight-nav-source-test)

    add_executable(forgelight-geometry-source-test
        tests/forgelight_geometry_source_test.cpp)
    target_link_libraries(forgelight-geometry-source-test PRIVATE
        forgelight-nav-source)
    target_include_directories(forgelight-geometry-source-test PRIVATE
        ${RECAST_DIR}/Recast/Include)
    if(MSVC)
        target_compile_options(forgelight-geometry-source-test PRIVATE /W4 /O2)
    else()
        target_compile_options(forgelight-geometry-source-test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME forgelight-geometry-source-contract
        COMMAND forgelight-geometry-source-test)

    add_executable(navigation-transitions-test
        tests/navigation_transitions_test.cpp)
    target_link_libraries(navigation-transitions-test PRIVATE
        forgelight-nav-source)
    if(MSVC)
        target_compile_options(navigation-transitions-test PRIVATE /W4 /O2)
    else()
        target_compile_options(navigation-transitions-test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME navigation-transitions-contract
        COMMAND navigation-transitions-test)

    add_test(NAME tilecache-overlay-self-test
        COMMAND tilecache-overlay --self-test)

    add_test(NAME tilecache-polycount-histogram
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DOVERLAY=$<TARGET_FILE:tilecache-overlay>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/tilecache-polycount
            -P ${CMAKE_SOURCE_DIR}/tests/verify_polycount_histogram.cmake)

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

    add_test(NAME semantic-buried-obstacle-on-slope
        COMMAND navmesh-builder
            ${CMAKE_SOURCE_DIR}/tests/semantic_buried_obstacle_on_slope.obj
            ${CMAKE_BINARY_DIR}/semantic-buried-obstacle-on-slope/output.bin
            --profile human
            --agent-climb 0.5
            --region-min 1
            --region-merge 1
            --dynamic-door-obstacles
            --verify-baked-semantics
            --semantic-report ${CMAKE_BINARY_DIR}/semantic-buried-obstacle-on-slope/semantics.json
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

    add_test(NAME semantic-report-publication
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/semantic-report-publication
            -P ${CMAKE_SOURCE_DIR}/tests/verify_semantic_report_publication.cmake
    )

    add_test(NAME global-bounds-overlay-contract
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DOVERLAY=$<TARGET_FILE:tilecache-overlay>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/global-bounds-overlay
            -P ${CMAKE_SOURCE_DIR}/tests/verify_global_bounds_overlay.cmake
    )

    add_test(NAME tilecache-completeness-fail-closed
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/tilecache-completeness
            -P ${CMAKE_SOURCE_DIR}/tests/verify_tilecache_completeness.cmake
    )

    add_test(NAME direct-nav-completeness-fail-closed
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/direct-nav-completeness
            -P ${CMAKE_SOURCE_DIR}/tests/verify_direct_nav_completeness.cmake
    )

    add_test(NAME geometry-source-determinism
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/geometry-source-determinism
            -P ${CMAKE_SOURCE_DIR}/tests/verify_geometry_source_determinism.cmake
    )

    add_test(NAME parallel-build-determinism
        COMMAND ${CMAKE_COMMAND}
            -DBUILDER=$<TARGET_FILE:navmesh-builder>
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT_DIR=${CMAKE_BINARY_DIR}/parallel-build-determinism
            -P ${CMAKE_SOURCE_DIR}/tests/verify_parallel_determinism.cmake
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
