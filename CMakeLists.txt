cmake_minimum_required(VERSION 3.29)

project(LegoHKM VERSION 0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ompl REQUIRED)
if (ompl_FOUND)
    message(STATUS "Found OMPL: ${ompl_DIR}")
else ()
    message(FATAL_ERROR "OMPL not found")
endif ()

add_subdirectory(libs/IK-lib)

set(SOURCE_FILES
    src/main.cpp
    src/path_finding.cpp
)

add_executable(LegoHKM ${SOURCE_FILES})
target_include_directories(LegoHKM PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(LegoHKM PRIVATE ompl IKObjectFiles IK-lib)

# add_compile_options(-Wall -Wconversion -g -fstack-usage -DEXTRA_CALC -O0 -lm)
