cmake_minimum_required(VERSION 2.6)

PROJECT( H264v2 )

message("Source Dir:" ${H264v2_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${H264v2_SOURCE_DIR}/../bin)
SET(LIBRARY_OUTPUT_PATH ${H264v2_SOURCE_DIR}/../lib)
SET(RUNTIME_OUTPUT_DIRECTORY ${H264v2_SOURCE_DIR}/../bin)

SET(H264v2_BIN ${H264v2_SOURCE_DIR}/../bin)

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
# 64bit
  message("Detected 64-bit build - compiling with -fPIC")
  SET(CMAKE_CXX_FLAGS "-fPIC")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
# 32 bit
message("Detected 32-bit build")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

if (USE_GLOBAL_VPP)
message("Using global VPP lib to build H264v2")
set(VPP_DIR
${GLOBAL_VPP_DIR}
)
else (USE_GLOBAL_VPP)
message("Using local VPP lib to build H264v2")
set(VPP_DIR
${H264v2_SOURCE_DIR}/libs/VppLib
)
endif(USE_GLOBAL_VPP)

# CMakeLists.txt in Codec Utils dir
INCLUDE_DIRECTORIES(
${H264v2_SOURCE_DIR}/../include
${H264v2_SOURCE_DIR}/../include/H264v2
${H264v2_SOURCE_DIR}/../include/H264v2Codec
${VPP_DIR}/include
${VPP_DIR}/include/CodecUtils
${VPP_DIR}/include/GeneralUtils
${VPP_DIR}/include/ImageUtils
${VPP_DIR}/include/StreamUtils
)

# Lib directories
LINK_DIRECTORIES(
lib
${VPP_DIR}/lib
)

SET(H264v2_LIB_HDRS
../include/H264v2/H264v2.h
../include/H264v2Codec/H264v2Codec.h
../include/H264v2Codec/H264v2CodecHeader.h
../include/H264v2/stdafx.h
)

SET(H264v2_LIB_SRCS 
./H264v2/H264v2.cpp
./H264v2Codec/H264v2Codec.cpp
./H264v2Codec/H264v2CodecHeader.cpp
./H264v2/stdafx.cpp
)

if (USE_GLOBAL_VPP)
else(USE_GLOBAL_VPP)
ADD_SUBDIRECTORY(${VPP_DIR}/src)
endif (USE_GLOBAL_VPP)

ADD_LIBRARY( H264v2 SHARED ${H264v2_LIB_SRCS} ${H264v2_LIB_HDRS})

TARGET_LINK_LIBRARIES(
H264v2
VppLib
) 

