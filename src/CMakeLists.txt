FILE(COPY juliet-server DESTINATION ${CMAKE_BINARY_DIR})
FILE(REMOVE ${CMAKE_BINARY_DIR}/juliet-server/src/gen)

add_subdirectory(chan-gen-types)

add_library(robot STATIC
    robot.cpp
    lego/lego_model.cpp
    lego/motors.cpp
    lego/lego_robot.cpp
    visualisation-integration/vis_robot.cpp
)
target_link_libraries(robot PUBLIC
    robotics_cape
    ${Boost_LIBRARIES}
    chan-static
    chan-comms
    IK-lib
)

set(SOURCE_FILES
    IMotion.cpp
    eigen_kinematics.cpp
    juliet_comms.cpp
    robot_thread.cpp
    trajectory_planner.cpp
    client.cpp
    calibration/calibration_parsing.cpp
)

set(LOCAL_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)

if (USE_OMPL)
    list(APPEND SOURCE_FILES ompl_pathfinding.cpp)
else()
    message(STATUS "Using custom path finding")
    list(APPEND SOURCE_FILES custom_pathfinding.cpp)
endif()

list(APPEND SOURCE_FILES main.cpp)

#[[ if(BUILD_PROD)
    # list(APPEND SOURCE_FILES board_main.cpp)
    list(APPEND SOURCE_FILES main.cpp)
else()
    list(APPEND SOURCE_FILES dev_main.cpp)
endif() ]]

# LegoHKM
add_executable(LegoHKM ${SOURCE_FILES})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
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
    chan-static
    chan-comms
    robot
)

target_compile_features(LegoHKM PRIVATE cxx_std_17)

# Calibration Program

add_executable(Calibration calibration/calibration_program.cpp)
target_link_libraries(Calibration PUBLIC
    robotics_cape
)

# Jogging Program

add_executable(Jogging 
jogging/jogging_program.cpp
calibration/calibration_parsing.cpp
eigen_kinematics.cpp
IMotion.cpp
custom_pathfinding.cpp
)
target_link_libraries(Jogging PUBLIC
    robotics_cape
    IK-lib
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    robot
)

# JL Emulation

find_package(Threads REQUIRED)

add_executable(jl_emulator 
visualisation-integration/jl_emulation.cpp
)
target_link_libraries(jl_emulator PUBLIC
    chan-static
    chan-comms
    Threads::Threads
)
