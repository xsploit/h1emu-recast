if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")

# Direct navmesh tiles used to be inserted into dtNavMesh from inside the
# OpenMP tile loop, so tile slots/refs -- and therefore serialized bytes --
# depended on thread completion order. This regression intentionally runs
# WITH parallelism enabled (unlike geometry-source-determinism, which pins
# OMP_NUM_THREADS=1) on a multi-tile grid, and requires every artifact to be
# byte-identical across repeated identical builds. A small tile size forces
# a grid of many tiles so OpenMP scheduling variance is actually exercised.
set(ENV{OMP_NUM_THREADS} 8)

set(runs run-a run-b run-c)
foreach(run ${runs})
  file(MAKE_DIRECTORY "${OUTPUT_DIR}/${run}")
  execute_process(
    COMMAND "${BUILDER}"
      "${SOURCE_DIR}/tests/bake_semantics.obj"
      "${OUTPUT_DIR}/${run}/output.bin"
      --profile human
      --tile-size 16
      --region-min 1
      --region-merge 1
      --dynamic-door-obstacles
      --semantic-report "${OUTPUT_DIR}/${run}/semantics.json"
    RESULT_VARIABLE run_result
    OUTPUT_VARIABLE run_stdout
    ERROR_VARIABLE run_stderr
  )
  if(NOT run_result EQUAL 0)
    message(FATAL_ERROR
      "Parallel determinism ${run} failed (${run_result}):\n"
      "${run_stdout}${run_stderr}")
  endif()
endforeach()

file(GLOB reference_artifacts RELATIVE "${OUTPUT_DIR}/run-a"
  "${OUTPUT_DIR}/run-a/z1_*.bin")
list(APPEND reference_artifacts semantics.json)
list(LENGTH reference_artifacts artifact_count)
if(artifact_count LESS 3)
  message(FATAL_ERROR
    "Expected at least one navmesh part, one cache part, and the semantic "
    "report; found only ${artifact_count} artifact(s): "
    "${reference_artifacts}")
endif()

foreach(artifact ${reference_artifacts})
  file(SHA256 "${OUTPUT_DIR}/run-a/${artifact}" reference_hash)
  foreach(run run-b run-c)
    if(NOT EXISTS "${OUTPUT_DIR}/${run}/${artifact}")
      message(FATAL_ERROR "${run} is missing artifact ${artifact}")
    endif()
    file(SHA256 "${OUTPUT_DIR}/${run}/${artifact}" run_hash)
    if(NOT run_hash STREQUAL reference_hash)
      message(FATAL_ERROR
        "${artifact} differed between identical parallel builds "
        "(run-a=${reference_hash} ${run}=${run_hash}) -- the multithreaded "
        "tile pipeline is not deterministic")
    endif()
  endforeach()
endforeach()
