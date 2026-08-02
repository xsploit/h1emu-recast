if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/success")
file(MAKE_DIRECTORY "${OUTPUT_DIR}/overflow")

# First prove the success path reports equality through actual serialization.
execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/bake_semantics.obj"
    "${OUTPUT_DIR}/success/output.bin"
    --profile human
    --region-min 1
    --region-merge 1
    --dynamic-door-obstacles
  RESULT_VARIABLE success_result
  OUTPUT_VARIABLE success_stdout
  ERROR_VARIABLE success_stderr
)
set(success_output "${success_stdout}${success_stderr}")
if(NOT success_result EQUAL 0)
  message(FATAL_ERROR
    "Completeness success fixture failed (${success_result}):\n${success_output}")
endif()
if(NOT success_output MATCHES
    "Cache completeness: generated=3 inserted=3 serializable=3 serialized=3")
  message(FATAL_ERROR
    "Success fixture did not prove equal completeness counters:\n${success_output}")
endif()
if(NOT EXISTS "${OUTPUT_DIR}/success/z1_cache_0.bin")
  message(FATAL_ERROR "Success fixture did not serialize a TileCache artifact")
endif()

# Then exceed the production 16-layer-per-column capacity with a deterministic
# 17-floor column. This exercises a real dtTileCache::addTile failure, not a
# mock, and must return failure before either navmesh artifact family is opened.
execute_process(
  COMMAND "${BUILDER}"
    "${SOURCE_DIR}/tests/tilecache_layer_capacity_overflow.obj"
    "${OUTPUT_DIR}/overflow/output.bin"
    --profile human
    --cell-size 1
    --cell-height 0.1
    --agent-radius 0
    --agent-climb 0.5
    --tile-size 16
    --region-min 1
    --region-merge 1
    --global-bounds 0 -1 0 16 50 16
  RESULT_VARIABLE overflow_result
  OUTPUT_VARIABLE overflow_stdout
  ERROR_VARIABLE overflow_stderr
)
set(overflow_output "${overflow_stdout}${overflow_stderr}")
if(overflow_result EQUAL 0)
  message(FATAL_ERROR
    "Layer-capacity fixture unexpectedly succeeded:\n${overflow_output}")
endif()
if(NOT overflow_output MATCHES "\\[ERROR\\] tileCache->addTile failed")
  message(FATAL_ERROR
    "Layer-capacity fixture failed for the wrong reason:\n${overflow_output}")
endif()
if(NOT overflow_output MATCHES "generated=17 inserted=16 maxTiles=16")
  message(FATAL_ERROR
    "Layer-capacity counters were not deterministic:\n${overflow_output}")
endif()
if(EXISTS "${OUTPUT_DIR}/overflow/z1_0.bin" OR
   EXISTS "${OUTPUT_DIR}/overflow/z1_cache_0.bin")
  message(FATAL_ERROR
    "Fail-closed fixture wrote a partial navmesh or TileCache artifact")
endif()
