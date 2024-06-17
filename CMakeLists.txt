cmake_minimum_required(VERSION 3.29)

project(OMPLTesting VERSION 0.1)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(ompl REQUIRED)
if (ompl_FOUND)
    message(STATUS "Found OMPL: ${ompl_DIR}")
else ()
    message(FATAL_ERROR "OMPL not found")
endif ()

add_executable(ompltest src/main.cpp)
target_include_directories(ompltest PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ompltest PRIVATE ompl)
