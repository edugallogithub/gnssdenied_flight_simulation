cmake_minimum_required(VERSION 3.6)			# Cmake version check
project(phd)								# create project "phd"
set(CMAKE_CXX_STANDARD 11)            		# enable c++11 standard

# ONLY ONE LINE ACTIVE BELOW
#set(CMAKE_BUILD_TYPE debug CACHE STRING "Choose debug or release." FORCE)
set(CMAKE_BUILD_TYPE release CACHE STRING "Choose debug or release." FORCE)
IF (WIN32)
	set(CMAKE_CXX_FLAGS "/MP /EHsc" CACHE STRING "Value given by jmake. Posible override though env variable." FORCE)
	set(CMAKE_CXX_FLAGS_DEBUG "/MDd /ZI /Od" CACHE STRING "Value given by jmake. Posible override though env variable." FORCE)
	set(CMAKE_CXX_FLAGS_RELEASE "/MD" CACHE STRING "Value given by jmake. Posible override though env variable." FORCE)
ELSE()
	set(CMAKE_CXX_FLAGS "-fPIC -fvisibility=hidden -fvisibility-inlines-hidden " CACHE STRING "Add if required.." FORCE)
	set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3" CACHE STRING "Debug options." FORCE)
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0" CACHE STRING "Release options." FORCE)
ENDIF()
set(DEBUG_TARGET_SUFFIX "d" CACHE STRING "Add to targets as suffix.")

set(PROJECTS_SRC_ROOT ${CMAKE_SOURCE_DIR}) # full path to top level of current CMake source tree
set(PROJECTS_BIN_ROOT ${CMAKE_BINARY_DIR}) # full path to top level of current CMake build tree
set(BUILD_SANDBOX ${PROJECTS_BIN_ROOT}/sandbox) # sandbox path one level below build tree
file(MAKE_DIRECTORY ${BUILD_SANDBOX}) # create sandbox directory (and its parents if required)
file(WRITE ${BUILD_SANDBOX}/hook "this file is for cmake to find this directory") # create "hook" file within sandbox
find_path(EXECUTABLE_OUTPUT_PATH hook ${BUILD_SANDBOX}) # find path where "hook" file is located
find_path(LIBRARY_OUTPUT_PATH hook ${BUILD_SANDBOX}) # find path where "hook" file is located
link_directories(${LIBRARY_OUTPUT_PATH}) # specify paths where linker will look for libraries
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH}) # set cache variable equal to environment variable

#=================================================================================================================
# Build Detection 32/64-bit
#=================================================================================================================
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(BUILD_ARCH 64)
    set(BUILD_ARCH_SUFFIX "")
    message("-- Detected 64-bit Build")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(BUILD_ARCH 32)
    set(BUILD_ARCH_SUFFIX "32")
    set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB32_PATHS TRUE)
    message("-- Notice! Detected 32-bit Build")
else()
    message(FATAL_ERROR "-- Unknown Build Architecture: contact with your administrator")
endif()

#=================================================================================================================
# Eigen
#=================================================================================================================
if (NOT EIGEN_FOUND)
    find_package(EIGEN REQUIRED) # finds EIGEN library and sets EIGEN_FOUND to true, mandatory
	include_directories(${EIGEN_INCLUDE_DIR}) # adds input directory to those compiler uses to search for include files.
	include_directories(${EIGEN_INCLUDE_DIRS}) # adds input directory to those compiler uses to search for include files.
endif (NOT EIGEN_FOUND)
if (NOT EIGEN_FOUND)
	message(SEND_ERROR "'EIGEN' is mandatory") # writes message, continues processing, but skips generation
endif (NOT EIGEN_FOUND )

# Following lines are for Windows compatibility
set(OPENCV_DIR $ENV{OPENCV_DIR})
# End Windows compatibility

if (NOT OPENCV_FOUND)
	find_package(OPENCV REQUIRED)
	include_directories(${OPENCV_INCLUDE_DIR})	
	include_directories(${OPENCV_INCLUDE_DIRS})
	message("-- Found OPENCV -I: " ${OPENCV_INCLUDE_DIRS}) 
endif (NOT OPENCV_FOUND)
IF (NOT OPENCV_FOUND)
	message(SEND_ERROR "'OPENCV' is mandatory")
ENDIF (NOT OPENCV_FOUND )

#if (NOT G2O_FOUND)
#	find_package(G2O REQUIRED)
#	include_directories(${G2O_INCLUDE_DIRS})
#endif (NOT G2O_FOUND)
#if (NOT G2O_FOUND)
#	message(SEND_ERROR "'G2O' is mandatory")
#endif (NOT G2O_FOUND )

#=================================================================================================================
# Use Snippet for boost
#=================================================================================================================
message("-- Using boost DYNAMIC MD libraries!")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_USE_MULTITHREADED ON)
add_definitions(-DBOOST_ALL_DYN_LINK)

IF (WIN32)
if(BUILD_ARCH EQUAL 32)
    set(Boost_LIBRARY_DIR $ENV{BOOST_ROOT}/stage/win32/lib)
	message ("-- Added Boost_LIBRARY_DIR Windows 32 bits = ${Boost_LIBRARY_DIR}")
elseif()
    set(Boost_LIBRARY_DIR $ENV{BOOST_ROOT}/stage/x64/lib)
	message ("-- Added Boost_LIBRARY_DIR Windows 64 bits = ${Boost_LIBRARY_DIR}")
endif()
ENDIF()

IF (WIN32)
	find_package( Boost 1.56 COMPONENTS date_time filesystem system program_options regex graph REQUIRED)
ELSE()
	find_package( Boost 1.56 COMPONENTS date_time filesystem thread system program_options regex graph REQUIRED)
ENDIF()

message("-- Found BOOST -I: " ${Boost_INCLUDE_DIRS}) 
message("-- Found BOOST -L: " ${Boost_LIBRARIES})

IF(Boost_FOUND)
	include_directories( ${Boost_INCLUDE_DIRS} )
	link_directories( ${Boost_LIBRARY_DIRS} )
	IF (WIN32 AND CMAKE_SYSTEM_VERSION)
		if( ${CMAKE_SYSTEM_VERSION} EQUAL 6.1 ) # Windows 7
			message ("-- Added BOOST Windows 7 platform: -D_WIN32_WINNT=0x0601")
			add_definitions(-D_WIN32_WINNT=0x0601)
		elseif( ${CMAKE_SYSTEM_VERSION} EQUAL 6.2 ) # Windows 8
			message ("-- Added BOOST Windows 8 platform: -D_WIN32_WINNT=0x0602")
			add_definitions(-D_WIN32_WINNT=0x0602)
		elseif( ${CMAKE_SYSTEM_VERSION} EQUAL 6.3 ) # Windows 8.1
			message ("-- Added BOOST Windows 8.1 platform: -D_WIN32_WINNT=0x0603")
			add_definitions(-D_WIN32_WINNT=0x0603)
		elseif( ${CMAKE_SYSTEM_VERSION} EQUAL 10.0 ) # Windows 10
			message ("-- Added BOOST Windows 10 platform: -D_WIN32_WINNT=0x0A00")
			add_definitions(-D_WIN32_WINNT=0x0A00)
		else() # Some other Windows
			message(WARNING "No windows version detected for BOOST...")
		endif()
	ENDIF()
ENDIF()

add_subdirectory(phd) # adds subdirectory to look for source CMakeLists.txt files










