cmake_minimum_required(VERSION 3.0)

project(LegoHKM VERSION 0.1)

option(BUILD_PROD "Build for BeagleBoard" OFF)
option(USE_OMPL "Build with OMPL library" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)

if (USE_OMPL)
    add_compile_options(-DUSE_OMPL)

    set(OMPL_BUILD_TESTS OFF)
    set(OMPL_BUILD_PYBINDINGS OFF)
    set(OMPL_BUILD_PYTESTS OFF)
    set(OMPL_BUILD_TESTS OFF)
    set(Boost_USE_MULTITHREADED ON)  

    find_package(ompl REQUIRED)
    if (ompl_FOUND)
        message(STATUS "Found OMPL: ${OMPL_INCLUDE_DIRS}")
    else ()
        message(FATAL_ERROR "OMPL not found")
    endif ()
endif()


if(BUILD_PROD)
    # add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=0 -static -w)
    # set(Boost_USE_STATIC_LIBS   ON)
else()
    add_subdirectory(libs/chan)

    add_compile_options(-D DEV)
endif()

find_package(Boost REQUIRED COMPONENTS thread timer chrono serialization filesystem system program_options)
if (Boost_FOUND)
    message(STATUS "Found boost: ${Boost_DIR}")
    message(STATUS "Boost libs: ${Boost_LIBRARIES}")
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

add_subdirectory(src)

if (NOT BUILD_PROD)
    add_subdirectory(tests)
endif()
