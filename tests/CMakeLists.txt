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
)

target_link_libraries(custom_pathfinding_test PRIVATE Eigen3::Eigen IK-lib)

add_test(
  NAME custom_pathfinding_test
  COMMAND $<TARGET_FILE:custom_pathfinding_test>
  )
