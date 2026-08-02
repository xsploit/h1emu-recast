if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(MAKE_DIRECTORY "${OUTPUT_DIR}")
set(INPUT_ABSOLUTE "${SOURCE_DIR}/tests/semantic_contract.obj")
set(INPUT_RELATIVE "./tests/../tests/semantic_contract.obj")
set(REPORT_ABSOLUTE "${OUTPUT_DIR}/absolute.json")
set(REPORT_RELATIVE "${OUTPUT_DIR}/relative.json")

execute_process(
    COMMAND "${BUILDER}" "${INPUT_ABSOLUTE}" "${OUTPUT_DIR}/absolute.bin"
        --validate-semantics-only --require-all-semantics
        --dynamic-door-obstacles --semantic-report "${REPORT_ABSOLUTE}"
    RESULT_VARIABLE ABSOLUTE_RESULT
)
if(NOT ABSOLUTE_RESULT EQUAL 0)
    message(FATAL_ERROR "absolute-path semantic report failed")
endif()

execute_process(
    COMMAND "${BUILDER}" "${INPUT_RELATIVE}" "${OUTPUT_DIR}/relative.bin"
        --validate-semantics-only --require-all-semantics
        --dynamic-door-obstacles --semantic-report "${REPORT_RELATIVE}"
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE RELATIVE_RESULT
)
if(NOT RELATIVE_RESULT EQUAL 0)
    message(FATAL_ERROR "relative-path semantic report failed")
endif()

file(SHA256 "${REPORT_ABSOLUTE}" ABSOLUTE_SHA256)
file(SHA256 "${REPORT_RELATIVE}" RELATIVE_SHA256)
if(NOT ABSOLUTE_SHA256 STREQUAL RELATIVE_SHA256)
    message(FATAL_ERROR
        "semantic report depends on caller path: ${ABSOLUTE_SHA256} != ${RELATIVE_SHA256}")
endif()

file(READ "${REPORT_ABSOLUTE}" REPORT_CONTENT)
file(SHA256 "${INPUT_ABSOLUTE}" INPUT_SHA256)
if(NOT REPORT_CONTENT MATCHES "\"inputSha256\": \"${INPUT_SHA256}\"")
    message(FATAL_ERROR "semantic report did not bind the source SHA-256")
endif()
string(FIND "${REPORT_CONTENT}" "\"inputPath\"" INPUT_PATH_INDEX)
if(NOT INPUT_PATH_INDEX EQUAL -1)
    message(FATAL_ERROR "semantic report leaked caller-spelled inputPath")
endif()
if(NOT REPORT_CONTENT MATCHES "\"bakedSemanticsVerified\": false")
    message(FATAL_ERROR "validation-only report did not fail closed")
endif()
string(FIND "${REPORT_CONTENT}" "\"artifacts\": [  ]" ARTIFACTS_INDEX)
if(ARTIFACTS_INDEX EQUAL -1)
    message(FATAL_ERROR "validation-only report unexpectedly bound output artifacts")
endif()

message(STATUS "path-independent semantic report SHA256=${ABSOLUTE_SHA256}")
