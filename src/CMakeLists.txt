project(ClearMOT)

# build_type configuration
if (CMAKE_BUILD_TYPE)
	string(TOUPPER ${CMAKE_BUILD_TYPE} _build_type)
	set(CMAKE_BUILD_TYPE ${_build_type} CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE)

# Build_type should DEBUG or RELEASE
if (CMAKE_BUILD_TYPE STREQUAL "")
	message("No build type specified: forcing to RELEASE")
	set(CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE STREQUAL "")

add_definitions(-Wall)

if (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	add_definitions(-ggdb)
elseif (CMAKE_BUILD_TYPE STREQUAL "RELEASE")
	add_definitions(-O3)
else (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	message("Invalid build type specified: doing nothing")
endif (CMAKE_BUILD_TYPE STREQUAL "DEBUG")

set(EXECUTABLE_OUTPUT_PATH ../bin)

find_package(OpenCV 2.4.2 REQUIRED)

#search for libxml2
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})

cmake_minimum_required(VERSION 2.8)

add_executable(ClearMOT main.cpp clearMOT.cpp munkres.cpp)
target_link_libraries(ClearMOT ${OpenCV_LIBS} ${LIBXML2_LIBRARIES})
