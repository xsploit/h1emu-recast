if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/run-a")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/run-b")

# ObjGeometrySource::queryTile() must produce the same tile-local vertex
# remap and triangle selection on every call for identical input bounds.
# Force single-threaded execution: the tile-building loop is parallelized
# with OpenMP, and running the pre-existing multithreaded direct-build path
# repeatedly on this same fixture was independently observed to be
# nondeterministic (a real, separately-tracked defect unrelated to this
# seam). Pinning OMP_NUM_THREADS=1 here isolates ObjGeometrySource's own
# determinism from that unrelated concurrency issue.
set(ENV{OMP_NUM_THREADS} 1)

execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${OUTPUT_DIR}/run-a/output.bin"
    --profile human
    --region-min 1
    --region-merge 1
    --dynamic-door-obstacles
    --verify-baked-semantics
    --semantic-report "${OUTPUT_DIR}/run-a/semantics.json"
  RESULT_VARIABLE result_a
  OUTPUT_VARIABLE stdout_a
  ERROR_VARIABLE stderr_a
)
if(NOT result_a EQUAL 0)
  message(FATAL_ERROR "Run A failed (${result_a}):\n${stdout_a}${stderr_a}")
endif()

execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${OUTPUT_DIR}/run-b/output.bin"
    --profile human
    --region-min 1
    --region-merge 1
    --dynamic-door-obstacles
    --verify-baked-semantics
    --semantic-report "${OUTPUT_DIR}/run-b/semantics.json"
  RESULT_VARIABLE result_b
  OUTPUT_VARIABLE stdout_b
  ERROR_VARIABLE stderr_b
)
if(NOT result_b EQUAL 0)
  message(FATAL_ERROR "Run B failed (${result_b}):\n${stdout_b}${stderr_b}")
endif()

foreach(artifact z1_0.bin z1_cache_0.bin semantics.json)
  set(path_a "${OUTPUT_DIR}/run-a/${artifact}")
  set(path_b "${OUTPUT_DIR}/run-b/${artifact}")
  if(NOT EXISTS "${path_a}" OR NOT EXISTS "${path_b}")
    message(FATAL_ERROR "Expected artifact missing: ${artifact}")
  endif()
  file(SHA256 "${path_a}" hash_a)
  file(SHA256 "${path_b}" hash_b)
  if(NOT hash_a STREQUAL hash_b)
    message(FATAL_ERROR
      "${artifact} differed between identical single-threaded runs "
      "(a=${hash_a} b=${hash_b}) -- ObjGeometrySource::queryTile() is not "
      "deterministic")
  endif()
endforeach()
