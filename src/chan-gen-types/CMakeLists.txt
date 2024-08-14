set(CHAN_GEN_TYPES_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(CHAN_DEF_FILES
    ${CHAN_GEN_TYPES_DIR}/defs/motion_types.jl
)

set(CHAN_GEN_FILES_CXX
    ${CMAKE_BINARY_DIR}/gen_c/motion_types.c
)
set(CHAN_GEN_FILES_OTHER
    ${CMAKE_BINARY_DIR}/juliet-server/src/gen_types/motion_types.jlt
)

set(COMPILED_OUTPUT_DIR ${CMAKE_BINARY_DIR})
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/gen_c)

add_custom_command(
    OUTPUT ${CHAN_GEN_FILES_CXX} ${CHAN_GEN_FILES_OTHER}
    COMMAND ${CHAN_GEN_TYPES_DIR}/regen.sh ${COMPILED_OUTPUT_DIR}
    WORKING_DIRECTORY ${CHAN_GEN_TYPES_DIR}
    DEPENDS ${CHAN_DEF_FILES} ${CHAN_GEN_TYPES_DIR}/regen.sh
    COMMENT "Regenerating files using regen.sh because defs/ files changed"
    VERBATIM
)

add_library(chan-comms STATIC ${CHAN_GEN_FILES_CXX})
target_link_libraries(chan-comms PUBLIC chan)
