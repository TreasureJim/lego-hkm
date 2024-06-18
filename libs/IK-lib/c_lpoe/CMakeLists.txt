cmake_minimum_required(VERSION 3.10)

project(c_lpoe)

set(SOURCE_FILES
	lpoe.c
	matrix_math.c
	test_lpoe.c
	test_matrix_math.c
)

set(HEADER_FILES
	lpoe.h
	matrix_math.h
)

add_library(c_lpoe STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(c_lpoe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
