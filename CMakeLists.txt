PROJECT(morph)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

IF (CMAKE_MAJOR_VERSION EQUAL 2 AND NOT CMAKE_MINOR_VERSION LESS 6)
	cmake_policy( SET CMP0003 NEW )
	cmake_policy( SET CMP0005 OLD )
ENDIF (CMAKE_MAJOR_VERSION EQUAL 2 AND NOT CMAKE_MINOR_VERSION LESS 6)

MESSAGE (STATUS "${PROJECT_NAME} version ${VERSION}")

# Set where to find our internal CMake scripts
SET (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")

# Enable -Wall by default unless on Win or Solaris
IF (NOT PROJECT_OS_WIN AND NOT PROJECT_OS_SOLARIS)
    SET (WALL "-Wno-write-strings" )
ENDIF (NOT PROJECT_OS_WIN AND NOT PROJECT_OS_SOLARIS)

if(APPLE)
    #SET( FORCE_ARCH "-arch i386" )
endif(APPLE)

SET (CMAKE_CXX_FLAGS_RELEASE "-fPIC ${FORCE_ARCH} ${OPTIMIZE} -DSTAGE -DNDEBUG ${WALL} " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_CXX_FLAGS_DEBUG "-fPIC -ggdb  ${FORCE_ARCH} ${WALL} -DSTAGE" CACHE INTERNAL "C Flags for debug" FORCE)
SET (CMAKE_CXX_FLAGS_PROFILE "-fPIC -ggdb -pg  ${FORCE_ARCH} ${WALL} -DSTAGE" CACHE INTERNAL "C Flags for profile" FORCE)
SET (CMAKE_C_FLAGS_RELEASE "-fPIC ${FORCE_ARCH} ${OPTIMIZE} -DSTAGE -DNDEBUG ${WALL} " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_C_FLAGS_DEBUG "-fPIC -ggdb  ${FORCE_ARCH} ${WALL} -DSTAGE" CACHE INTERNAL "C Flags for debug" FORCE)
SET (CMAKE_C_FLAGS_PROFILE "-fPIC -ggdb -pg  ${FORCE_ARCH} ${WALL} -DSTAGE" CACHE INTERNAL "C Flags for profile" FORCE)



#####################################
# Set the default build type
IF (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE "debug" CACHE STRING 
    "Choose the type of build, options are: release (default) debug profile" FORCE)
ENDIF (NOT CMAKE_BUILD_TYPE)
STRING(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

MESSAGE( STATUS "Build type ${CMAKE_BUILD_TYPE}" )

ENABLE_TESTING()

SET(ROBOT_BASE_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/base")
SET(ROBOT_BASE_LIBRARY_DIR "${PROJECT_BINARY_DIR}/base")
SET(ROBOT_BASE morphBase)
SET(ROBOT_BASE_LIBRARY ${ROBOT_BASE_LIBRARY_DIR}/lib${ROBOT_BASE}.a)


ADD_SUBDIRECTORY(base)
#if (NOT APPLE)
    #ADD_SUBDIRECTORY(robot3d)
    #endif (NOT APPLE)
ADD_SUBDIRECTORY(stage)
ADD_SUBDIRECTORY(canvas)
