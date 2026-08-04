if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/rejected")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/diagnostic")

set(common_options
  --profile human
  --cell-size 1
  --cell-height 0.1
  --agent-radius 0
  --agent-climb 0.5
  --tile-size 16
  --region-min 1
  --region-merge 1
  --global-bounds 0 -1 0 32 5 16
  --direct-nav-tile-bits 0)

execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/direct_nav_capacity_overflow.obj"
    "${OUTPUT_DIR}/rejected/output.bin"
    ${common_options}
  RESULT_VARIABLE rejected_result
  OUTPUT_VARIABLE rejected_stdout
  ERROR_VARIABLE rejected_stderr)
set(rejected_output "${rejected_stdout}${rejected_stderr}")
if(rejected_result EQUAL 0)
  message(FATAL_ERROR
    "Direct-capacity fixture unexpectedly succeeded:\n${rejected_output}")
endif()
if(NOT rejected_output MATCHES
   "Direct tiles : generated=2 admitted=1 rejected=1 empty=0 total=2")
  message(FATAL_ERROR
    "Direct-capacity counters were not exact:\n${rejected_output}")
endif()
if(NOT rejected_output MATCHES
   "Refusing to write a partial direct navmesh")
  message(FATAL_ERROR
    "Direct-capacity failure was not fail-closed:\n${rejected_output}")
endif()
if(EXISTS "${OUTPUT_DIR}/rejected/z1_0.bin" OR
   EXISTS "${OUTPUT_DIR}/rejected/z1_cache_0.bin")
  message(FATAL_ERROR "Fail-closed direct fixture wrote partial artifacts")
endif()

execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/direct_nav_capacity_overflow.obj"
    "${OUTPUT_DIR}/diagnostic/output.bin"
    ${common_options}
    --allow-partial-navmesh
  RESULT_VARIABLE diagnostic_result
  OUTPUT_VARIABLE diagnostic_stdout
  ERROR_VARIABLE diagnostic_stderr)
set(diagnostic_output "${diagnostic_stdout}${diagnostic_stderr}")
if(NOT diagnostic_result EQUAL 0)
  message(FATAL_ERROR
    "Explicit diagnostic override failed:\n${diagnostic_output}")
endif()
if(NOT diagnostic_output MATCHES
   "Diagnostic partial-navmesh override accepted")
  message(FATAL_ERROR
    "Explicit diagnostic override was not reported:\n${diagnostic_output}")
endif()
if(NOT EXISTS "${OUTPUT_DIR}/diagnostic/z1_0.bin" OR
   NOT EXISTS "${OUTPUT_DIR}/diagnostic/z1_cache_0.bin")
  message(FATAL_ERROR "Diagnostic override did not write both artifacts")
endif()
