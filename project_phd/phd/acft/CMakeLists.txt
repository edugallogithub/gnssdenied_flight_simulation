
set(prjname acft)

file (GLOB dir_10_headers *.h         *.hpp)
file (GLOB dir_11_headers acft/*.h    acft/*.hpp)
file (GLOB dir_12_headers control/*.h control/*.hpp)
file (GLOB dir_13_headers guid/*.h    guid/*.hpp)
file (GLOB dir_15_headers sens/*.h    sens/*.hpp)
file (GLOB dir_16_headers st/*.h      st/*.hpp)

source_group(""        FILES ${dir_10_headers})
source_group("acft"    FILES ${dir_11_headers})
source_group("control" FILES ${dir_12_headers})
source_group("guid"    FILES ${dir_13_headers})
source_group("sens"    FILES ${dir_15_headers})
source_group("st"      FILES ${dir_16_headers})
set(all_headers ${dir_10_headers} ${dir_11_headers} ${dir_12_headers} ${dir_13_headers} ${dir_15_headers} ${dir_16_headers} )

file (GLOB dir_10_sources *.cpp         *.c         *.cxx)
file (GLOB dir_11_sources acft/*.cpp    acft/*.c    acft/*.cxx)
file (GLOB dir_12_sources control/*.cpp control/*.c control/*.cxx)
file (GLOB dir_13_sources guid/*.cpp    guid/*.c    guid/*.cxx)
file (GLOB dir_15_sources sens/*.cpp    sens/*.c    sens/*.cxx)
file (GLOB dir_16_sources st/*.cpp      st/*.c      st/*.cxx)
source_group(""        FILES ${dir_10_sources})
source_group("acft"    FILES ${dir_11_sources})
source_group("control" FILES ${dir_12_sources})
source_group("guid"    FILES ${dir_13_sources})
source_group("sens"    FILES ${dir_15_sources})
source_group("st"      FILES ${dir_16_sources})
set(all_sources ${dir_10_sources} ${dir_11_sources} ${dir_12_sources} ${dir_13_sources} ${dir_15_sources} ${dir_16_sources} )

string(TOUPPER ${prjname} PRJNAME)
add_definitions(-D${PRJNAME}_DLL)
add_definitions(-D${PRJNAME}_DLL_EXPORTS)

add_library(${prjname} SHARED ${all_headers} ${all_sources})

set_target_properties(${prjname} PROPERTIES DEBUG_OUTPUT_NAME ${prjname}${DEBUG_TARGET_SUFFIX})
set_target_properties(${prjname} PROPERTIES RELEASE_OUTPUT_NAME ${prjname})

include_directories(${PROJECTS_SRC_ROOT}/phd)

# Following lines are for Windows compatibility
add_definitions(-DATTITUDE_DLL)
# End Windows compatibility
target_link_libraries(${prjname} debug math${DEBUG_TARGET_SUFFIX} optimized math)
target_link_libraries(${prjname} debug ang${DEBUG_TARGET_SUFFIX} optimized ang)
target_link_libraries(${prjname} debug env${DEBUG_TARGET_SUFFIX} optimized env)
target_link_libraries(${prjname} ${Boost_LIBRARIES})

foreach(A_LIB ${EIGEN_LIBS_RELEASE}) 
	target_link_libraries(${prjname} optimized ${A_LIB}) # adds all Eigen libraries to target
endforeach()
foreach(A_LIB ${EIGEN_LIBS_DEBUG}) 
	target_link_libraries(${prjname} debug ${A_LIB}) # adds all Eigen libraries to target
endforeach()





