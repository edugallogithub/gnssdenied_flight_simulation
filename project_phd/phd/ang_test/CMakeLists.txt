
set(prjname ang_test) # set cache variable

file (GLOB dir_8_headers *.h *.hpp) # generates list of files that match expressions and store it into variable
source_group("" FILES ${dir_8_headers}) # define a group into which sources are placed for IDE project generation
set(all_headers ${dir_8_headers}) # set cache variable with headers

file (GLOB dir_9_sources *.cpp *.c)
SOURCE_GROUP("" FILES ${dir_9_sources})
set(all_sources ${dir_9_sources}) # set cache variable with sources

add_executable(${prjname} ${all_headers} ${all_sources})

set_target_properties(${prjname} PROPERTIES DEBUG_OUTPUT_NAME ${prjname}${DEBUG_TARGET_SUFFIX})
set_target_properties(${prjname} PROPERTIES RELEASE_OUTPUT_NAME ${prjname})

include_directories(${PROJECTS_SRC_ROOT}/phd)
 
target_link_libraries(${prjname} debug math${DEBUG_TARGET_SUFFIX} optimized math)
target_link_libraries(${prjname} debug ang${DEBUG_TARGET_SUFFIX} optimized ang)
target_link_libraries(${prjname} debug jail${DEBUG_TARGET_SUFFIX} optimized jail)
target_link_libraries(${prjname} ${Boost_LIBRARIES})

foreach(A_LIB ${EIGEN_LIBS_RELEASE}) 
	target_link_libraries(${prjname} optimized ${A_LIB})
endforeach()
foreach(A_LIB ${EIGEN_LIBS_DEBUG}) 
	target_link_libraries(${prjname} debug ${A_LIB})
endforeach()

