FILE(COPY juliet-server DESTINATION ${CMAKE_BINARY_DIR})
FILE(REMOVE ${CMAKE_BINARY_DIR}/juliet-server/src/gen)

add_subdirectory(chan-gen-types)

set(SOURCE_FILES
    IMotion.cpp
    lego_model.cpp
    motors.cpp
    robot.cpp
    lego_robot.cpp
    eigen_kinematics.cpp
    juliet_comms.cpp
    robot_thread.cpp
    trajectory_planner.cpp
    client.cpp
    visualisation-integration/vis_robot.cpp
    calibration/calibration_parsing.cpp
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

# LegoHKM
add_executable(LegoHKM ${SOURCE_FILES})

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${LOCAL_LIBS_DIR}/ompl/src
    ${LOCAL_LIBS_DIR}/librobotcontrol/library/include
    ${LOCAL_LIBS_DIR}/chan/include
    ${LOCAL_LIBS_DIR}/IK-lib
    ${CMAKE_BINARY_DIR}/gen_c

    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(LegoHKM PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    IK-lib
    robotics_cape
    chan-static
    chan-comms
)

target_compile_features(LegoHKM PRIVATE cxx_std_17)

# Motor Control

add_executable(MotorControl motorcontrol.cpp motors.cpp lego_model.cpp)
target_link_libraries(MotorControl PUBLIC
    ${Boost_LIBRARIES}
    robotics_cape
)

# Calibration Program

add_executable(Calibration calibration/calibration_program.cpp)
target_link_libraries(Calibration PUBLIC
    robotics_cape
)

# Jogging Program

add_executable(Jogging 
jogging/jogging_program.cpp
motors.cpp
calibration/calibration_parsing.cpp
lego_model.cpp
)
target_link_libraries(Jogging PUBLIC
    robotics_cape
    IK-lib
    ${Boost_LIBRARIES}
)
