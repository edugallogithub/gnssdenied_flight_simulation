
set(prjname ang) # set cache variable

file (GLOB dir_0_headers *.h *.hpp) # generates list of files that match expressions and store it into variable
file (GLOB dir_3_headers transform/*.h transform/*.hpp)
file (GLOB dir_4_headers rotate/*.h rotate/*.hpp)
source_group("" FILES ${dir_0_headers}) # define a group into which sources are placed for IDE project generation
source_group("transform" FILES ${dir_3_headers})
source_group("rotate" FILES ${dir_4_headers})
set(all_headers ${dir_0_headers} ${dir_3_headers} ${dir_4_headers}) # set cache variable with headers

file (GLOB dir_0_sources *.cpp *.c)
file (GLOB dir_3_sources transform/*.cpp transform/*.c transform/*.cxx)
file (GLOB dir_4_sources rotate/*.cpp rotate/*.c rotate/*.cxx)
source_group("" FILES ${dir_0_sources})
source_group("transform" FILES ${dir_3_sources})
source_group("rotate" FILES ${dir_4_sources})
set(all_sources ${dir_0_sources} ${dir_3_sources} ${dir_4_sources}) # set cache variable with sources

string(TOUPPER ${prjname} PRJNAME) # converts prjname to uppercase and loads it into PRJNAME
add_definitions(-D${PRJNAME}_DLL) # adds -D define flags to the compilation of source files
add_definitions(-D${PRJNAME}_DLL_EXPORTS) # adds -D define flags to the compilation of source files

add_library(${prjname} SHARED ${all_headers} ${all_sources}) # adds library target as SHARED (linked dinamically, shared at runtime)

set_target_properties(${prjname} PROPERTIES DEBUG_OUTPUT_NAME ${prjname}${DEBUG_TARGET_SUFFIX}) # add target property so debug target has the d suffix
set_target_properties(${prjname} PROPERTIES RELEASE_OUTPUT_NAME ${prjname}) # add target property so release target coincides with project

include_directories(${PROJECTS_SRC_ROOT}/phd) # adds input directory to those compiler uses to search for include files.

target_link_libraries(${prjname} debug math${DEBUG_TARGET_SUFFIX} optimized math)
foreach(A_LIB ${EIGEN_LIBS_RELEASE}) 
	target_link_libraries(${prjname} optimized ${A_LIB}) # adds all Eigen libraries to target
endforeach()
foreach(A_LIB ${EIGEN_LIBS_DEBUG}) 
	target_link_libraries(${prjname} debug ${A_LIB}) # adds all Eigen libraries to target
endforeach()
