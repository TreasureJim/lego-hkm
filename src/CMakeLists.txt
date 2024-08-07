set(CHAN_GEN_TYPES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/chan-gen-types)

set(CHAN_DEF_FILES
    ${CHAN_GEN_TYPES_DIR}/defs/motion_types.jl
)
set(CHAN_GEN_FILES 
    ${CHAN_GEN_TYPES_DIR}/gen_c/motion_types.c
)

add_custom_command(
    OUTPUT ${CHAN_GEN_FILES}
    COMMAND ${CHAN_GEN_TYPES_DIR}/regen.sh
    WORKING_DIRECTORY ${CHAN_GEN_TYPES_DIR}
    DEPENDS ${CHAN_DEF_FILES} ${CHAN_GEN_TYPES_DIR}/regen.sh
    COMMENT "Regenerating files using regen.sh because defs/ files changed"
    VERBATIM
)

set(SOURCE_FILES
    mark2_0_fixed.cpp
    motors.cpp
    robot.cpp
    eigen_kinematics.cpp
    ${CHAN_GEN_FILES}
)

list(APPEND SOURCE_FILES 
  ${CHAN_GEN_FILES}
)

set(LOCAL_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)

if (USE_OMPL)
    list(APPEND SOURCE_FILES ompl_pathfinding.cpp)
else()
    message(STATUS "Using custom path finding")
    list(APPEND SOURCE_FILES custom_pathfinding.cpp)
endif()

if(BUILD_PROD)
    # list(APPEND SOURCE_FILES board_main.cpp)
    list(APPEND SOURCE_FILES main.cpp)
else()
    list(APPEND SOURCE_FILES dev_main.cpp)
endif()

add_executable(LegoHKM ${SOURCE_FILES})

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${LOCAL_LIBS_DIR}/ompl/src
    ${LOCAL_LIBS_DIR}/librobotcontrol/library/include
    ${LOCAL_LIBS_DIR}/chan/include
    ${CHAN_GEN_TYPES_DIR}/gen_c

    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(LegoHKM PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    IK-lib
    robotics_cape
    chan-static
)

target_compile_features(LegoHKM PRIVATE cxx_std_17)
