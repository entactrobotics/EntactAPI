cmake_minimum_required (VERSION 2.6)
project (EntactAPI)

SET( EntactAPI_MAJOR_VERSION 1 )
SET( EntactAPI_MINOR_VERSION 0 )
SET( EntactAPI_BUILD_VERSION 1 )

SET( EntactAPI_FULL_VERSION
${EntactAPI_MAJOR_VERSION}.${EntactAPI_MINOR_VERSION}.${EntactAPI_BUILD_VERSION} ) 

# Check for stdint.h
INCLUDE (CheckIncludeFiles)
CHECK_INCLUDE_FILES("stdint.h" HAVE_STDINT_H)

# set headers and sources
SET( EntactAPI_HEADERS "${EntactAPI_SOURCE_DIR}/EntactAPI.h"
                     "${EntactAPI_SOURCE_DIR}/EAPI_packets.h"
                     "${EntactAPI_SOURCE_DIR}/EAPI_kinematics.h" )
SET( EntactAPI_SOURCES "${EntactAPI_SOURCE_DIR}/EntactAPI.cpp" )

# EntactAPI shared library definition.
ADD_LIBRARY(EntactAPI SHARED ${EntactAPI_SOURCES} ${EntactAPI_HEADERS})
