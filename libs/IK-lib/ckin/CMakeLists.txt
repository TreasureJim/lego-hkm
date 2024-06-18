cmake_minimum_required(VERSION 3.10)

project(ckin)

set(SOURCE_FILES
	common.c
	dh.c
	example_params.c
	kin_skel.c
	kin_skel_trace.c
	lin_alg.c
	lpoe_invkin.c
)

set(HEADER_FILES
	common.h
	debug.h
	dh.h
	example_params.h
	kin_skel.h
	lin_alg.h
	lpoe_invkin.h
)

add_library(ckin STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(ckin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../c_lpoe)
