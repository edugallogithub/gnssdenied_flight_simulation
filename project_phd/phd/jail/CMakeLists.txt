
set(prjname jail)

file (GLOB dir_31_headers *.h *.hpp)
source_group("" FILES ${dir_31_headers})
set(all_headers ${dir_31_headers})

file (GLOB dir_32_sources *.cpp *.c)
source_group("" FILES ${dir_32_sources})
set(all_sources ${dir_32_sources})

string(TOUPPER ${prjname} PRJNAME)
add_definitions(-D${PRJNAME}_DLL)
add_definitions(-D${PRJNAME}_DLL_EXPORTS)

add_library(${prjname} SHARED ${all_headers} ${all_sources})

set_target_properties(${prjname} PROPERTIES DEBUG_OUTPUT_NAME ${prjname}${DEBUG_TARGET_SUFFIX})
set_target_properties(${prjname} PROPERTIES RELEASE_OUTPUT_NAME ${prjname})

include_directories(${PROJECTS_SRC_ROOT}/phd)

