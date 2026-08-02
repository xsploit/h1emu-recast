if(NOT DEFINED BUILDER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "BUILDER, SOURCE_DIR, and OUTPUT_DIR are required")
endif()

file(MAKE_DIRECTORY "${OUTPUT_DIR}")
execute_process(
    COMMAND "${BUILDER}"
        "${SOURCE_DIR}/tests/bounded_semantic_coverage.obj"
        "${OUTPUT_DIR}/output.bin"
        --profile human
        --region-min 1
        --region-merge 1
        --bounds 0.1 0.1 11.9 11.9
        --verify-baked-semantics
        --semantic-report "${OUTPUT_DIR}/semantics.json"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE stdout
    ERROR_VARIABLE stderr
)

set(log "${stdout}\n${stderr}")
if(NOT result EQUAL 0)
    message(FATAL_ERROR "bounded semantic bake failed (${result}):\n${log}")
endif()
if(NOT log MATCHES "Semantic verification source coverage: area4=2 \\(requested bounds\\)")
    message(FATAL_ERROR "expected only two in-bounds interior source triangles:\n${log}")
endif()
if(log MATCHES "Semantic verification source coverage:[^\n]*area1=" OR
   log MATCHES "Semantic verification source coverage:[^\n]*area6=")
    message(FATAL_ERROR "out-of-bounds terrain or vertical ramp was required:\n${log}")
endif()
if(NOT log MATCHES "direct semantic polygons:[^\n]*area4=[1-9][0-9]*[^\n]*PASS" OR
   NOT log MATCHES "tilecache semantic polygons:[^\n]*area4=[1-9][0-9]*[^\n]*PASS")
    message(FATAL_ERROR "direct and TileCache inspection did not retain area 4:\n${log}")
endif()
