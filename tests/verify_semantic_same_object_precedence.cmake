if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

set(input "${SOURCE_DIR}/tests/semantic_same_object_precedence.obj")
set(run_a "${OUTPUT_DIR}/run-a")
set(run_b "${OUTPUT_DIR}/run-b")
file(MAKE_DIRECTORY "${run_a}" "${run_b}")

foreach(run_dir IN ITEMS "${run_a}" "${run_b}")
    execute_process(
        COMMAND "${BUILDER}"
            "${input}"
            "${run_dir}/output.bin"
            --profile human
            --agent-radius 0
            --agent-climb 0.5
            --region-min 1
            --region-merge 1
            --dynamic-door-obstacles
            --verify-baked-semantics
            --semantic-report "${run_dir}/semantics.json"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE stdout
        ERROR_VARIABLE stderr
    )
    set(log "${stdout}\n${stderr}")
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "semantic precedence bake failed (${result}):\n${log}")
    endif()
    if(NOT log MATCHES "Semantic verification source coverage: area2=7 area3=8 area4=5 area5=4 area7=2")
        message(FATAL_ERROR "semantic parser did not expose the expected source coverage:\n${log}")
    endif()
    foreach(label IN ITEMS direct tilecache)
        if(NOT log MATCHES "${label} semantic polygons: total=11 area2=1 area3=2 area4=2 area5=3 area7=3 obstacleProbes=1 PASS")
            message(FATAL_ERROR "${label} did not preserve semantic priority and carve behavior:\n${log}")
        endif()
    endforeach()
    if(run_dir STREQUAL run_a)
        set(log_a "${log}")
    else()
        set(log_b "${log}")
    endif()
endforeach()

file(READ "${run_a}/semantics.json" semantic_report)
if(NOT semantic_report MATCHES "\"sourceTriangles\": 31" OR
   NOT semantic_report MATCHES "\"keptTriangles\": 28" OR
   NOT semantic_report MATCHES "\"excludedTriangles\": 3" OR
   NOT semantic_report MATCHES "\"nav_door_panel_dynamic\": 1" OR
   NOT semantic_report MATCHES "\"nav_exclude\": 2" OR
   NOT semantic_report MATCHES "\"nav_obstacle_static\": 2")
    message(FATAL_ERROR "semantic parser did not preserve exclude/carve provenance:\n${semantic_report}")
endif()

file(SHA256 "${run_a}/semantics.json" report_a_sha)
file(SHA256 "${run_b}/semantics.json" report_b_sha)
if(NOT report_a_sha STREQUAL report_b_sha)
    message(FATAL_ERROR "semantic reports are nondeterministic: ${report_a_sha} != ${report_b_sha}")
endif()

foreach(label IN ITEMS direct tilecache)
    string(REGEX MATCH "${label} semantic polygons:[^\n]*" histogram_a "${log_a}")
    string(REGEX MATCH "${label} semantic polygons:[^\n]*" histogram_b "${log_b}")
    if(histogram_a STREQUAL "" OR NOT histogram_a STREQUAL histogram_b)
        message(FATAL_ERROR "${label} semantic classification is nondeterministic:\n${histogram_a}\n${histogram_b}")
    endif()
endforeach()

file(GLOB cache_a "${run_a}/z1_cache_*.bin")
file(GLOB cache_b "${run_b}/z1_cache_*.bin")
list(SORT cache_a)
list(SORT cache_b)
list(LENGTH cache_a cache_a_count)
list(LENGTH cache_b cache_b_count)
if(cache_a_count EQUAL 0 OR NOT cache_a_count EQUAL cache_b_count)
    message(FATAL_ERROR "deterministic runs emitted different cache part counts")
endif()
math(EXPR cache_last "${cache_a_count} - 1")
foreach(index RANGE 0 ${cache_last})
    list(GET cache_a ${index} part_a)
    list(GET cache_b ${index} part_b)
    file(SHA256 "${part_a}" part_a_sha)
    file(SHA256 "${part_b}" part_b_sha)
    if(NOT part_a_sha STREQUAL part_b_sha)
        message(FATAL_ERROR "cache part ${index} is nondeterministic: ${part_a_sha} != ${part_b_sha}")
    endif()
endforeach()

file(GLOB nav_a "${run_a}/z1_[0-9]*.bin")
file(GLOB nav_b "${run_b}/z1_[0-9]*.bin")
list(SORT nav_a)
list(SORT nav_b)
list(LENGTH nav_a nav_a_count)
list(LENGTH nav_b nav_b_count)
if(nav_a_count EQUAL 0 OR NOT nav_a_count EQUAL nav_b_count)
    message(FATAL_ERROR "deterministic runs emitted different direct navmesh part counts")
endif()
math(EXPR nav_last "${nav_a_count} - 1")
foreach(index RANGE 0 ${nav_last})
    list(GET nav_a ${index} part_a)
    list(GET nav_b ${index} part_b)
    file(SHA256 "${part_a}" part_a_sha)
    file(SHA256 "${part_b}" part_b_sha)
    if(NOT part_a_sha STREQUAL part_b_sha)
        message(FATAL_ERROR "direct navmesh part ${index} is nondeterministic: ${part_a_sha} != ${part_b_sha}")
    endif()
endforeach()

message(STATUS "same-object semantic precedence and deterministic navmesh bytes PASS")
