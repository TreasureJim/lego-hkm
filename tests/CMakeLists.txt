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

# UNFINISHED TEST

# Visualises planned path given from path planner
add_library(custom_pathfinding_vis_test SHARED
    custom_pathfinding_vis_test.cpp
    ${SOURCE_DIR}/eigen_kinematics.cpp
    ${SOURCE_DIR}/custom_pathfinding.cpp
    ${SOURCE_DIR}/mark2_0_fixed.cpp
    ${SOURCE_DIR}/test_helpers.cpp
)

target_link_libraries(custom_pathfinding_vis_test PRIVATE Eigen3::Eigen IK-lib)

# add_test(
#   NAME custom_pathfinding_vis_test
#   COMMAND $<TARGET_FILE:custom_pathfinding_test>
#   )
