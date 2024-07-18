cmake_minimum_required(VERSION 3.29)

project(LegoHKM VERSION 0.1)

option(BUILD_PROD "Build for BeagleBoard" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)

set(SOURCE_FILES
    src/math/3d_circle.cpp
    src/path_finding.cpp
    src/spacial_conv.cpp
    src/mark2_0_fixed.cpp
    src/motors.cpp
    src/robot.cpp
)

set(OMPL_BUILD_TESTS OFF)
set(OMPL_BUILD_PYBINDINGS OFF)
set(OMPL_BUILD_PYTESTS OFF)
set(OMPL_BUILD_TESTS OFF)

if(BUILD_PROD)
    find_package(ompl REQUIRED PATHS /opt/ompl-arm)
    # if (OMPL_FOUND)
    #     message(STATUS "Found OMPL: ${OMPL_INCLUDE_DIRS}")
    # else ()
    #     message(FATAL_ERROR "OMPL not found")
    # endif ()
    # set(OMPL_PATH libs/ompl)
    # add_subdirectory(${OMPL_PATH})
    # set(OMPL_INCLUDE_DIRS ${OMPL_PATH}/src)
    # target_include_directories(ompl PUBLIC ../)
    list(APPEND SOURCE_FILES src/board_main.cpp)

    add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=0 -static -w)
    set(Boost_USE_STATIC_LIBS   ON)
else()
    find_package(ompl REQUIRED)
    if (OMPL_FOUND)
        message(STATUS "Found OMPL: ${OMPL_INCLUDE_DIRS}")
    else ()
        message(FATAL_ERROR "OMPL not found")
    endif ()

    list(APPEND SOURCE_FILES src/dev_main.cpp)
    add_compile_options(-D DEV)
endif()

set(Boost_USE_MULTITHREADED ON)  
find_package(Boost REQUIRED COMPONENTS thread serialization filesystem system program_options)
if (Boost_FOUND)
    message(STATUS "Found boost: ${Boost_DIR}")
else ()
    message(FATAL_ERROR "boost not found")
endif ()

find_package (Eigen3 REQUIRED NO_MODULE)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_DIR}")
else ()
    message(FATAL_ERROR "Eigen3 not found")
endif ()

add_subdirectory(libs/IK-lib)
add_subdirectory(libs/librobotcontrol)

file(GLOB BOOST_A "/opt/boost-arm/lib/*.a")

add_executable(LegoHKM ${SOURCE_FILES})
target_include_directories(LegoHKM PRIVATE 
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/ompl/src

    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(LegoHKM PRIVATE
    Eigen3::Eigen

    ${Boost_LIBRARIES}
    ${BOOST_A}

    # ${OMPL_LIBRARIES}
    /opt/ompl-arm/share/libompl.a

    IK-lib
    robotics_cape
)
