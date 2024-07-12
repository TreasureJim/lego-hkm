cmake_minimum_required(VERSION 3.29)

project(LegoHKM VERSION 0.1)

option(BUILD_PROD "Build for BeagleBoard" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(Boost_USE_MULTITHREADED ON)  
find_package(Boost REQUIRED COMPONENTS thread)
if (Boost_FOUND)
    message(STATUS "Found boost: ${boost_DIR}")
else ()
    message(FATAL_ERROR "boost not found")
endif ()

add_subdirectory(libs/IK-lib)
add_subdirectory(libs/librobotcontrol)

set(SOURCE_FILES
    src/math/3d_circle.cpp
    src/path_finding.cpp
    src/spacial_conv.cpp
    src/mark2_0_fixed.cpp
    src/motors.cpp
    src/robot.cpp
)

if(BUILD_PROD)
    add_subdirectory(libs/ompl)

    list(APPEND SOURCE_FILES src/board_main.cpp)
else()
find_package(ompl REQUIRED)
    if (ompl_FOUND)
        message(STATUS "Found OMPL: ${ompl_DIR}")
    else ()
        message(FATAL_ERROR "OMPL not found")
    endif ()

    list(APPEND SOURCE_FILES src/dev_main.cpp)
    add_compile_options(-D DEV)
endif()

add_executable(LegoHKM ${SOURCE_FILES})
target_include_directories(LegoHKM PRIVATE ${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})
target_link_libraries(LegoHKM PRIVATE Boost::thread ${Boost_LIBRARIES} ompl::ompl IK-lib robotics_cape)

# add_compile_options(-Wall -Wconversion -g -fstack-usage -DEXTRA_CALC -O0 -lm)
