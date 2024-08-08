enable_testing()

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(custom_pathfinding_test 
    custom_pathfinding_test.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/custom_pathfinding.cpp
    ${SOURCE_DIR}/mark2_0_fixed.cpp
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
    ${SOURCE_DIR}/mark2_0_fixed.cpp
    ${SOURCE_DIR}/test_helpers.cpp
)

target_link_libraries(custom_pathfinding_vis_test PRIVATE Eigen3::Eigen IK-lib)
target_include_directories(custom_pathfinding_vis_test PUBLIC ${EIGEN3_INCLUDE_DIR})
set_target_properties(custom_pathfinding_vis_test PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)
