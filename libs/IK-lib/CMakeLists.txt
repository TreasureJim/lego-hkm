cmake_minimum_required(VERSION 3.10)

project(IK-lib)

add_subdirectory(c_lpoe)
add_subdirectory(ckin)

set(SOURCE_FILES
	ax4_kinematics.c
	# calibrated_kinematics.c
	concept_prototype.c
	dyn.c
	HKM1800.c
	HKM1800_id.c
	kin_model.c
	kin_model_long.c
	kinematics.c
	mark2_0.c
	# mark2_0_calib.c
	# mark2_0_lpoe.c
	# mark2_0_perturbed.c
	myalloc.c

)

set(HEADER_FILES
	ax4_kinematics.h
	# calibrated_kinematics.h
	concept_prototype.h
	dyn.h
	HKM1800.h
	HKM1800_id.h
	kin_model.h
	kin_model_long.h
	kinematics.h
	mark2_0.h
	# mark2_0_calib.h
	# mark2_0_lpoe.h
	# mark2_0_perturbed.h
	myalloc.h
)

add_library(IK-lib STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(IK-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(IK-lib PRIVATE c_lpoe ckin)
