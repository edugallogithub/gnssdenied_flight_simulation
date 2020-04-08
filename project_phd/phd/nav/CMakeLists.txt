
set(prjname nav)

file (GLOB dir_10_headers *.h         *.hpp)
file (GLOB dir_11_headers motion/*.h  motion/*.hpp)
file (GLOB dir_12_headers kalman/*.h  kalman/*.hpp)
file (GLOB dir_13_headers att/*.h     att/*.hpp)
file (GLOB dir_14_headers air/*.h     air/*.hpp)
file (GLOB dir_15_headers gps/*.h     gps/*.hpp)
file (GLOB dir_16_headers pos/*.h     pos/*.hpp)
file (GLOB dir_17_headers nav/*.h     nav/*.hpp)
file (GLOB dir_17_headers init/*.h    init/*.hpp)
source_group(""        FILES ${dir_10_headers})
source_group("motion"  FILES ${dir_11_headers})
source_group("kalman"  FILES ${dir_12_headers})
source_group("att"     FILES ${dir_13_headers})
source_group("air"     FILES ${dir_14_headers})
source_group("gps"     FILES ${dir_15_headers})
source_group("pos"     FILES ${dir_16_headers})
source_group("nav"     FILES ${dir_17_headers})
source_group("nav"     FILES ${dir_18_headers})
set(all_headers ${dir_10_headers} ${dir_11_headers} ${dir_12_headers} ${dir_13_headers} ${dir_14_headers} ${dir_15_headers} ${dir_16_headers} ${dir_17_headers} ${dir_18_headers})

file (GLOB dir_10_sources *.cpp         *.c         *.cxx)
file (GLOB dir_11_sources motion/*.cpp  motion/*.c  motion/*.cxx)
file (GLOB dir_12_sources kalman/*.cpp  kalman/*.c  kalman/*.cxx)
file (GLOB dir_13_sources att/*.cpp     att/*.c     att/*.cxx)
file (GLOB dir_14_sources air/*.cpp     air/*.c     air/*.cxx)
file (GLOB dir_15_sources gps/*.cpp     gps/*.c     gps/*.cxx)
file (GLOB dir_16_sources pos/*.cpp     pos/*.c     pos/*.cxx)
file (GLOB dir_17_sources nav/*.cpp     nav/*.c     nav/*.cxx)
file (GLOB dir_18_sources init/*.cpp    init/*.c    init/*.cxx)
source_group(""        FILES ${dir_10_sources})
source_group("motion"  FILES ${dir_11_sources})
source_group("kalman"  FILES ${dir_12_sources})
source_group("att"     FILES ${dir_13_sources})
source_group("air"     FILES ${dir_14_sources})
source_group("gps"     FILES ${dir_15_sources})
source_group("pos"     FILES ${dir_16_sources})
source_group("nav"     FILES ${dir_17_sources})
source_group("init"    FILES ${dir_18_sources})
set(all_sources ${dir_10_sources} ${dir_11_sources} ${dir_12_sources} ${dir_13_sources} ${dir_14_sources} ${dir_15_sources} ${dir_16_sources} ${dir_17_sources} ${dir_18_sources})

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
target_link_libraries(${prjname} debug acft${DEBUG_TARGET_SUFFIX} optimized acft)
target_link_libraries(${prjname} ${Boost_LIBRARIES})

foreach(A_LIB ${EIGEN_LIBS_RELEASE}) 
	target_link_libraries(${prjname} optimized ${A_LIB}) # adds all Eigen libraries to target
endforeach()
foreach(A_LIB ${EIGEN_LIBS_DEBUG}) 
	target_link_libraries(${prjname} debug ${A_LIB}) # adds all Eigen libraries to target
endforeach()





