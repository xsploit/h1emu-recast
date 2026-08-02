cmake_minimum_required(VERSION 3.16)

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/base" "${OUTPUT_DIR}/regional" "${OUTPUT_DIR}/local")

set(COMMON_ARGS
    --profile human
    --cell-size 1
    --cell-height 0.5
    --agent-radius 0
    --agent-climb 0.5
    --tile-size 16
    --region-min 1
    --region-merge 1)

execute_process(
    COMMAND "${BUILDER}" "${SOURCE_DIR}/tests/global_bounds.obj"
            "${OUTPUT_DIR}/base/output.bin"
            ${COMMON_ARGS}
            --global-bounds 0 -1 0 64 3 64
    RESULT_VARIABLE BASE_RESULT
    OUTPUT_VARIABLE BASE_STDOUT
    ERROR_VARIABLE BASE_STDERR)
if(NOT BASE_RESULT EQUAL 0)
    message(FATAL_ERROR "global base bake failed:\n${BASE_STDOUT}\n${BASE_STDERR}")
endif()

execute_process(
    COMMAND "${BUILDER}" "${SOURCE_DIR}/tests/global_bounds.obj"
            "${OUTPUT_DIR}/regional/output.bin"
            ${COMMON_ARGS}
            --global-bounds 0 -1 0 64 3 64
            --bounds 0 0 48 48
    RESULT_VARIABLE REGIONAL_RESULT
    OUTPUT_VARIABLE REGIONAL_STDOUT
    ERROR_VARIABLE REGIONAL_STDERR)
if(NOT REGIONAL_RESULT EQUAL 0)
    message(FATAL_ERROR "global regional bake failed:\n${REGIONAL_STDOUT}\n${REGIONAL_STDERR}")
endif()

execute_process(
    COMMAND "${OVERLAY}"
            --base-dir "${OUTPUT_DIR}/base"
            --overlay-dir "${OUTPUT_DIR}/regional"
            --output-dir "${OUTPUT_DIR}/merged"
            --replace-coverage 1 1 2 2
            --overlay-coverage 0 0 3 3
    RESULT_VARIABLE MERGE_RESULT
    OUTPUT_VARIABLE MERGE_STDOUT
    ERROR_VARIABLE MERGE_STDERR)
if(NOT MERGE_RESULT EQUAL 0)
    message(FATAL_ERROR "compatible global overlay failed:\n${MERGE_STDOUT}\n${MERGE_STDERR}")
endif()

execute_process(
    COMMAND "${BUILDER}" "${SOURCE_DIR}/tests/global_bounds.obj"
            "${OUTPUT_DIR}/local/output.bin"
            ${COMMON_ARGS}
            --bounds 20 20 28 28
    RESULT_VARIABLE LOCAL_RESULT
    OUTPUT_VARIABLE LOCAL_STDOUT
    ERROR_VARIABLE LOCAL_STDERR)
if(NOT LOCAL_RESULT EQUAL 0)
    message(FATAL_ERROR "local regional bake failed:\n${LOCAL_STDOUT}\n${LOCAL_STDERR}")
endif()

execute_process(
    COMMAND "${OVERLAY}"
            --base-dir "${OUTPUT_DIR}/base"
            --overlay-dir "${OUTPUT_DIR}/local"
            --output-dir "${OUTPUT_DIR}/must-not-exist"
            --replace-coverage 1 1 2 2
            --overlay-coverage 0 0 3 3
    RESULT_VARIABLE MISMATCH_RESULT
    OUTPUT_VARIABLE MISMATCH_STDOUT
    ERROR_VARIABLE MISMATCH_STDERR)
if(MISMATCH_RESULT EQUAL 0)
    message(FATAL_ERROR "local-origin overlay unexpectedly succeeded")
endif()
if(NOT "${MISMATCH_STDERR}" MATCHES "parameter mismatch: (meshParams|cacheParams).orig")
    message(FATAL_ERROR "wrong local-origin diagnostic:\n${MISMATCH_STDERR}")
endif()

message(STATUS "global-bounds overlay contract PASS")
