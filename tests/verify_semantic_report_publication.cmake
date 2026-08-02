if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/verified")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/unverified")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/verification-failure")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/publication-failure")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/load-failure")

set(COMMON_ARGS
  --profile human
  --region-min 1
  --region-merge 1
  --dynamic-door-obstacles
)

# A report may claim verified only after direct and TileCache inspection and
# successful serialization. It binds the exact emitted part names and sizes.
set(VERIFIED_DIR "${OUTPUT_DIR}/verified")
set(VERIFIED_REPORT "${VERIFIED_DIR}/semantics.json")
execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${VERIFIED_DIR}/output.bin"
    ${COMMON_ARGS}
    --verify-baked-semantics
    --semantic-report "${VERIFIED_REPORT}"
  RESULT_VARIABLE VERIFIED_RESULT
  OUTPUT_VARIABLE VERIFIED_STDOUT
  ERROR_VARIABLE VERIFIED_STDERR
)
if(NOT VERIFIED_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Verified semantic report fixture failed (${VERIFIED_RESULT}):\n"
    "${VERIFIED_STDOUT}\n${VERIFIED_STDERR}")
endif()
if(NOT EXISTS "${VERIFIED_REPORT}")
  message(FATAL_ERROR "Verified bake did not publish its semantic report")
endif()
file(READ "${VERIFIED_REPORT}" VERIFIED_CONTENT)
if(NOT VERIFIED_CONTENT MATCHES "\"schemaVersion\": 2")
  message(FATAL_ERROR "Verified bake did not publish semantic report schema v2")
endif()
if(NOT VERIFIED_CONTENT MATCHES "\"bakedSemanticsVerified\": true")
  message(FATAL_ERROR "Verified bake report did not record successful inspection")
endif()
foreach(PART IN ITEMS z1_0.bin z1_cache_0.bin)
  if(NOT EXISTS "${VERIFIED_DIR}/${PART}")
    message(FATAL_ERROR "Verified bake did not produce ${PART}")
  endif()
  file(SIZE "${VERIFIED_DIR}/${PART}" PART_BYTES)
  file(SHA256 "${VERIFIED_DIR}/${PART}" PART_SHA256)
  if(NOT VERIFIED_CONTENT MATCHES
      "\"file\": \"${PART}\", \"bytes\": ${PART_BYTES}, \"identity\": \"fnv1a64:[0-9a-f]+\", \"sha256\": \"${PART_SHA256}\"")
    message(FATAL_ERROR
      "Verified report does not bind ${PART}, its size, and SHA-256")
  endif()
endforeach()

# Successful serialization without explicit baked inspection is useful only as
# runtime-only evidence and must record false.
set(UNVERIFIED_DIR "${OUTPUT_DIR}/unverified")
set(UNVERIFIED_REPORT "${UNVERIFIED_DIR}/semantics.json")
execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${UNVERIFIED_DIR}/output.bin"
    ${COMMON_ARGS}
    --semantic-report "${UNVERIFIED_REPORT}"
  RESULT_VARIABLE UNVERIFIED_RESULT
  OUTPUT_VARIABLE UNVERIFIED_STDOUT
  ERROR_VARIABLE UNVERIFIED_STDERR
)
if(NOT UNVERIFIED_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Unverified semantic report fixture failed (${UNVERIFIED_RESULT}):\n"
    "${UNVERIFIED_STDOUT}\n${UNVERIFIED_STDERR}")
endif()
file(READ "${UNVERIFIED_REPORT}" UNVERIFIED_CONTENT)
if(NOT UNVERIFIED_CONTENT MATCHES "\"bakedSemanticsVerified\": false")
  message(FATAL_ERROR "Unverified bake report did not fail closed")
endif()

# A deterministic erosion failure occurs during direct semantic inspection.
# A stale report is removed before the run and no replacement is published.
set(FAILURE_DIR "${OUTPUT_DIR}/verification-failure")
set(FAILURE_REPORT "${FAILURE_DIR}/semantics.json")
file(WRITE "${FAILURE_REPORT}" "{\"stale\":true}\n")
execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${FAILURE_DIR}/output.bin"
    ${COMMON_ARGS}
    --agent-radius 10
    --verify-baked-semantics
    --semantic-report "${FAILURE_REPORT}"
  RESULT_VARIABLE FAILURE_RESULT
  OUTPUT_VARIABLE FAILURE_STDOUT
  ERROR_VARIABLE FAILURE_STDERR
)
set(FAILURE_LOG "${FAILURE_STDOUT}\n${FAILURE_STDERR}")
if(FAILURE_RESULT EQUAL 0)
  message(FATAL_ERROR "Semantic verification failure fixture unexpectedly passed")
endif()
if(NOT FAILURE_LOG MATCHES "direct semantic inspection failed")
  message(FATAL_ERROR
    "Semantic verification fixture failed for the wrong reason:\n${FAILURE_LOG}")
endif()
if(EXISTS "${FAILURE_REPORT}" OR EXISTS "${FAILURE_REPORT}.tmp")
  message(FATAL_ERROR "Failed semantic inspection left a report publication")
endif()

# A report-publication failure after successful serialization must return
# nonzero and leave no complete-looking report or temp file.
set(PUBLICATION_DIR "${OUTPUT_DIR}/publication-failure")
set(BLOCKING_PARENT "${PUBLICATION_DIR}/not-a-directory")
set(BLOCKED_REPORT "${BLOCKING_PARENT}/semantics.json")
file(WRITE "${BLOCKING_PARENT}" "regular file\n")
execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${PUBLICATION_DIR}/output.bin"
    ${COMMON_ARGS}
    --verify-baked-semantics
    --semantic-report "${BLOCKED_REPORT}"
  RESULT_VARIABLE PUBLICATION_RESULT
  OUTPUT_VARIABLE PUBLICATION_STDOUT
  ERROR_VARIABLE PUBLICATION_STDERR
)
if(PUBLICATION_RESULT EQUAL 0)
  message(FATAL_ERROR "Semantic report publication failure unexpectedly passed")
endif()
if(EXISTS "${BLOCKED_REPORT}" OR EXISTS "${BLOCKED_REPORT}.tmp")
  message(FATAL_ERROR "Publication failure left a semantic report or temp file")
endif()

# Even a source-load failure clears prior evidence before parsing begins.
set(LOAD_FAILURE_DIR "${OUTPUT_DIR}/load-failure")
set(LOAD_FAILURE_REPORT "${LOAD_FAILURE_DIR}/semantics.json")
file(WRITE "${LOAD_FAILURE_REPORT}" "{\"stale\":true}\n")
execute_process(
  COMMAND "${BUILDER}"
    "${LOAD_FAILURE_DIR}/missing.obj"
    "${LOAD_FAILURE_DIR}/output.bin"
    --semantic-report "${LOAD_FAILURE_REPORT}"
  RESULT_VARIABLE LOAD_FAILURE_RESULT
)
if(LOAD_FAILURE_RESULT EQUAL 0)
  message(FATAL_ERROR "Missing source fixture unexpectedly passed")
endif()
if(EXISTS "${LOAD_FAILURE_REPORT}" OR EXISTS "${LOAD_FAILURE_REPORT}.tmp")
  message(FATAL_ERROR "Source-load failure retained stale semantic evidence")
endif()
