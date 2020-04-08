 
set(prjname exec_nav)

file (GLOB dir_01_headers *.h *.hpp)
source_group("" FILES ${dir_01_headers})
set(all_headers ${dir_01_headers} )

file (GLOB dir_01_sources *.cpp *.c)
source_group("" FILES ${dir_01_sources})
set(all_sources ${dir_01_sources} )

add_executable(${prjname} ${all_headers} ${all_sources})

set_target_properties(${prjname} PROPERTIES DEBUG_OUTPUT_NAME ${prjname}${DEBUG_TARGET_SUFFIX})
set_target_properties(${prjname} PROPERTIES RELEASE_OUTPUT_NAME ${prjname})

include_directories(${PROJECTS_SRC_ROOT}/phd)

# Following lines are for Windows compatibility
add_definitions(-DMATH_DLL)
add_definitions(-DATTITUDE_DLL)
add_definitions(-DEARTH_DLL)
add_definitions(-DFDYN_DLL)
add_definitions(-DJAIL_DLL)
# End Windows compatibility
target_link_libraries(${prjname} debug math${DEBUG_TARGET_SUFFIX} optimized math)
target_link_libraries(${prjname} debug ang${DEBUG_TARGET_SUFFIX} optimized ang)
target_link_libraries(${prjname} debug env${DEBUG_TARGET_SUFFIX} optimized env)
target_link_libraries(${prjname} debug acft${DEBUG_TARGET_SUFFIX} optimized acft)
target_link_libraries(${prjname} debug nav${DEBUG_TARGET_SUFFIX} optimized nav)
target_link_libraries(${prjname} ${Boost_LIBRARIES})

foreach(A_LIB ${EIGEN_LIBS_RELEASE}) 
	target_link_libraries(${prjname} optimized ${A_LIB})
endforeach()
foreach(A_LIB ${EIGEN_LIBS_DEBUG}) 
	target_link_libraries(${prjname} debug ${A_LIB})
endforeach()

