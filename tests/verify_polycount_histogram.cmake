if(NOT DEFINED BUILDER OR NOT DEFINED OVERLAY OR NOT DEFINED SOURCE_DIR OR
   NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR
    "BUILDER, OVERLAY, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/cache")

execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${OUTPUT_DIR}/cache/output.bin"
    --profile human
    --region-min 1
    --region-merge 1
    --dynamic-door-obstacles
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_stdout
  ERROR_VARIABLE build_stderr)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "Polycount cache fixture failed to build:\n${build_stdout}${build_stderr}")
endif()

execute_process(
  COMMAND "${OVERLAY}"
    --polycount-histogram
    --base-dir "${OUTPUT_DIR}/cache"
    --max-polys 32
    --report "${OUTPUT_DIR}/polycount.json"
  RESULT_VARIABLE pass_result
  OUTPUT_VARIABLE pass_stdout
  ERROR_VARIABLE pass_stderr)
set(pass_output "${pass_stdout}${pass_stderr}")
if(NOT pass_result EQUAL 0)
  message(FATAL_ERROR "Polycount pass fixture failed:\n${pass_output}")
endif()
if(NOT pass_output MATCHES "Polycount histogram PASS: all 3 layer\\(s\\)")
  message(FATAL_ERROR
    "Polycount pass fixture did not report all layers:\n${pass_output}")
endif()
if(NOT EXISTS "${OUTPUT_DIR}/polycount.json")
  message(FATAL_ERROR "Polycount pass fixture did not write its report")
endif()
file(READ "${OUTPUT_DIR}/polycount.json" report)
if(NOT report MATCHES "\"schema\":\"h1emu-tilecache-polycount-v1\"")
  message(FATAL_ERROR "Polycount report schema is missing")
endif()
foreach(field IN ITEMS zeroPolygons p50 p95 p99 maximum)
  if(NOT report MATCHES "\"${field}\":[0-9]+")
    message(FATAL_ERROR "Polycount report is missing numeric ${field}")
  endif()
endforeach()

execute_process(
  COMMAND "${OVERLAY}"
    --polycount-histogram
    --base-dir "${OUTPUT_DIR}/cache"
    --max-polys 0
  RESULT_VARIABLE fail_result
  OUTPUT_VARIABLE fail_stdout
  ERROR_VARIABLE fail_stderr)
set(fail_output "${fail_stdout}${fail_stderr}")
if(fail_result EQUAL 0)
  message(FATAL_ERROR
    "Zero-poly threshold unexpectedly passed:\n${fail_output}")
endif()
if(NOT fail_output MATCHES "Polycount histogram FAIL")
  message(FATAL_ERROR
    "Zero-poly threshold failed without the expected diagnostic:\n${fail_output}")
endif()
