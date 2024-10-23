enable_testing()

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR}/gen_c
  ${LOCAL_LIBS_DIR}/librobotcontrol/library/include
)

add_executable(custom_pathfinding_test 
    custom_pathfinding_test.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/custom_pathfinding.cpp
    ${SOURCE_DIR}/lego/lego_model.cpp
    ${SOURCE_DIR}/test_helpers.cpp
)

target_link_libraries(custom_pathfinding_test PRIVATE Eigen3::Eigen IK-lib)

add_test(
  NAME custom_pathfinding_test
  COMMAND $<TARGET_FILE:custom_pathfinding_test>
  )

# Builds library for visualising pathfinding
add_library(custom_pathfinding_vis_test SHARED
    custom_pathfinding_vis_test.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/custom_pathfinding.cpp
    ${SOURCE_DIR}/lego/lego_model.cpp
    ${SOURCE_DIR}/test_helpers.cpp
)

target_link_libraries(custom_pathfinding_vis_test PRIVATE Eigen3::Eigen IK-lib)
target_include_directories(custom_pathfinding_vis_test PUBLIC ${EIGEN3_INCLUDE_DIR})
set_target_properties(custom_pathfinding_vis_test PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)

# Test juliet communication queue

# Client
add_executable(juliet_comms_test
    ${SOURCE_DIR}/juliet_comms.cpp
    ${SOURCE_DIR}/client.cpp
    ${SOURCE_DIR}/IMotion.cpp
    ${SOURCE_DIR}/lego/lego_model.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/custom_pathfinding.cpp
    julia_communication_test.cpp
)
target_link_libraries(juliet_comms_test PUBLIC 
chan-static
chan-comms
IK-lib
)
set(LOCAL_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
target_include_directories(juliet_comms_test PUBLIC
    ${LOCAL_LIBS_DIR}/chan/include
    ${LOCAL_LIBS_DIR}/IK-lib
    ${CMAKE_BINARY_DIR}/gen_c
    ${LOCAL_LIBS_DIR}/librobotcontrol/library/include

    ${PROJECT_SOURCE_DIR}/include
)

# Server
add_executable(juliet_comms_server
  julia_comm_server_test.cpp
)
target_link_libraries(juliet_comms_server PUBLIC 
chan-static
chan-comms
)
set(LOCAL_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
target_include_directories(juliet_comms_server PUBLIC
    ${LOCAL_LIBS_DIR}/chan/include
    ${CMAKE_BINARY_DIR}/gen_c

    ${PROJECT_SOURCE_DIR}/include
)

# Visualisation connection test

add_executable(vis_control_test
    ${SOURCE_DIR}/IMotion.cpp
    ${SOURCE_DIR}/robot.cpp
    ${SOURCE_DIR}/visualisation-integration/vis_robot.cpp
    ${SOURCE_DIR}/test_helpers.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/custom_pathfinding.cpp
    vis_control_test.cpp
)
target_link_libraries(vis_control_test PUBLIC 
chan-static
chan-comms
IK-lib
)
set(LOCAL_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
target_include_directories(vis_control_test PUBLIC
    ${LOCAL_LIBS_DIR}/chan/include
    ${LOCAL_LIBS_DIR}/IK-lib
    ${CMAKE_BINARY_DIR}/gen_c
    ${LOCAL_LIBS_DIR}/librobotcontrol/library/include

    ${PROJECT_SOURCE_DIR}/include
)

# Calibration Parse Test

add_executable(calibration_parse_test
    calibration_parse_test.cpp
    ${SOURCE_DIR}/calibration/calibration_parsing.cpp
)
add_test(
  NAME calibration_parse_test
  COMMAND $<TARGET_FILE:calibration_parse_test>
  )

# Cart to Motor to Cart

add_executable(coord_test
    coord_test.cpp
    ${SOURCE_DIR}/calibration/calibration_parsing.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/lego/lego_model.cpp
    ${SOURCE_DIR}/test_helpers.cpp
    ${SOURCE_DIR}/lego/motors.cpp
)
target_link_libraries(coord_test PUBLIC
IK-lib
Eigen3::Eigen
robotics_cape
${Boost_LIBRARIES}
)
target_include_directories(coord_test PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR}/gen_c
  ${LOCAL_LIBS_DIR}/librobotcontrol/library/include
)
