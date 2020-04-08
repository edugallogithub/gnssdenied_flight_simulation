set(prjname math)

# ===== ===== HEADER FILES ===== =====
file (GLOB table_headers 	*.h 		*.hpp)				
file (GLOB interp_headers 	interp/*.h 	interp/*.hpp)
file (GLOB logic_headers 	logic/*.h 	logic/*.hpp)
file (GLOB vec_headers 		vec/*.h 	vec/*.hpp)
file (GLOB math_headers 	math/*.h 	math/*.hpp)
file (GLOB pred_headers 	pred/*.h 	pred/*.hpp)
file (GLOB pred0v_headers 	pred/pred0v/*.h	pred/pred0v/*.hpp)
file (GLOB pred1v_headers 	pred/pred1v/*.h pred/pred1v/*.hpp)
file (GLOB pred2v_headers 	pred/pred2v/*.h pred/pred2v/*.hpp)
file (GLOB pred3v_headers 	pred/pred3v/*.h pred/pred3v/*.hpp)
file (GLOB pred4v_headers 	pred/pred4v/*.h pred/pred4v/*.hpp)
file (GLOB templates_headers 	templates/*.h 	templates/*.hpp)

SOURCE_GROUP("" 			FILES ${table_headers})
SOURCE_GROUP("interp" 		FILES ${interp_headers})
SOURCE_GROUP("logic" 		FILES ${logic_headers})
SOURCE_GROUP("vec" 			FILES ${vec_headers})
SOURCE_GROUP("math" 		FILES ${math_headers})
SOURCE_GROUP("pred" 		FILES ${pred_headers})
SOURCE_GROUP("pred\\pred0v" FILES ${pred0v_headers})
SOURCE_GROUP("pred\\pred1v"	FILES ${pred1v_headers})
SOURCE_GROUP("pred\\pred2v" FILES ${pred2v_headers})
SOURCE_GROUP("pred\\pred3v" FILES ${pred3v_headers})
SOURCE_GROUP("pred\\pred4v" FILES ${pred4v_headers})
SOURCE_GROUP("templates" 	FILES ${templates_headers})
set(all_headers ${table_headers} ${interp_headers} ${logic_headers} ${pred_headers} ${pred0v_headers} ${pred1v_headers} ${pred2v_headers} ${pred3v_headers} ${pred4v_headers} ${vec_headers} ${math_headers} ${templates_headers} )

# ===== ===== SOURCE FILES ===== =====
file (GLOB table_sources 	*.cpp *.c)						
file (GLOB interp_sources 	interp/*.cpp 		interp/*.c 	interp/*.cxx)
file (GLOB logic_sources 	logic/*.cpp 		logic/*.c 	logic/*.cxx)
file (GLOB vec_sources 		vec/*.cpp 		vec/*.c 	vec/*.cxx)
file (GLOB math_sources 	math/*.cpp 		math/*.c 	math/*.cxx)
file (GLOB pred_sources 	pred/*.cpp 		pred/*.c 	pred/*.cxx)
file (GLOB pred0v_sources 	pred/pred0v/*.cpp 	pred/pred0v/*.c pred/pred0v/*.cxx)
file (GLOB pred1v_sources 	pred/pred1v/*.cpp 	pred/pred1v/*.c pred/pred1v/*.cxx)
file (GLOB pred2v_sources 	pred/pred2v/*.cpp 	pred/pred2v/*.c pred/pred2v/*.cxx)
file (GLOB pred3v_sources 	pred/pred3v/*.cpp 	pred/pred3v/*.c pred/pred3v/*.cxx)
file (GLOB pred4v_sources 	pred/pred4v/*.cpp 	pred/pred4v/*.c pred/pred4v/*.cxx)

SOURCE_GROUP("" 			FILES ${table_sources})
SOURCE_GROUP("interp"	 	FILES ${interp_sources})
SOURCE_GROUP("logic" 		FILES ${logic_sources})
SOURCE_GROUP("vec" 			FILES ${vec_sources})
SOURCE_GROUP("math" 		FILES ${math_sources})
SOURCE_GROUP("pred" 		FILES ${pred_sources})
SOURCE_GROUP("pred\\pred0v"	FILES ${pred0v_sources})
SOURCE_GROUP("pred\\pred1v"	FILES ${pred1v_sources})
SOURCE_GROUP("pred\\pred2v"	FILES ${pred2v_sources})
SOURCE_GROUP("pred\\pred3v"	FILES ${pred3v_sources})
SOURCE_GROUP("pred\\pred4v"	FILES ${pred4v_sources})
set(all_sources ${table_sources} ${interp_sources} ${logic_sources} ${vec_sources} ${math_sources} ${pred_sources} ${pred0v_sources} ${pred1v_sources} ${pred2v_sources} ${pred3v_sources} ${pred4v_sources})

string(TOUPPER ${prjname} PRJNAME) # converts prjname to uppercase and loads it into PRJNAME
add_definitions(-D${PRJNAME}_DLL) # adds -D define flags to the compilation of source files
add_definitions(-D${PRJNAME}_DLL_EXPORTS) # adds -D define flags to the compilation of source files

add_library(${prjname} SHARED ${all_headers} ${all_sources}) # adds library target as SHARED (linked dinamically, shared at runtime)

set_target_properties(${prjname} PROPERTIES DEBUG_OUTPUT_NAME ${prjname}${DEBUG_TARGET_SUFFIX}) # add target property so debug target has the d suffix
set_target_properties(${prjname} PROPERTIES RELEASE_OUTPUT_NAME ${prjname}) # add target property so release target coincides with project

include_directories(${PROJECTS_SRC_ROOT}/phd) # adds input directory to those compiler uses to search for include files.

target_link_libraries(${prjname} ${Boost_LIBRARIES})
foreach(A_LIB ${EIGEN_LIBS_RELEASE}) 
	target_link_libraries(${prjname} optimized ${A_LIB}) # adds all Eigen libraries to target
endforeach()
foreach(A_LIB ${EIGEN_LIBS_DEBUG}) 
	target_link_libraries(${prjname} debug ${A_LIB}) # adds all Eigen libraries to target
endforeach()



