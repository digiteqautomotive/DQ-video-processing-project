# CMakeLists.txt for <H264DecoderFilter> 

# # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS")

INCLUDE_DIRECTORIES(
${artistIncludes}
${artist_SOURCE_DIR}/../include/artist
${artistCodecIncludes}
${artistUtilIncludes}
${VPP_DIR}/include
${VPP_DIR}/include/CodecUtils
${VPP_DIR}/include/GeneralUtils
${VPP_DIR}/include/ImageUtils
${VPP_DIR}/include/StreamUtils
${vpp_SOURCE_DIR}/codecs/H264v2/include
${vpp_SOURCE_DIR}/codecs/H264v2/include/H264v2
${vpp_SOURCE_DIR}/codecs/H264v2/include/H264v2Codec
)
 
# Lib directories
LINK_DIRECTORIES(
${artistLink}
)

SET(FLT_HDRS
H264DecoderFilter.h
stdafx.h
VersionInfo.h
)

SET(FLT_SRCS 
DLLSetup.cpp
H264DecoderFilter.cpp
H264DecoderFilter.def
stdafx.cpp
)

ADD_LIBRARY(
H264DecoderFilter SHARED ${FLT_SRCS} ${FLT_HDRS})

TARGET_LINK_LIBRARIES (
H264DecoderFilter
${DsFilterLibs}
H264v2
) 

INSTALL(
  TARGETS H264DecoderFilter
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION bin
)

IF (REGISTER_DS_FILTERS)
ADD_CUSTOM_COMMAND(
TARGET H264DecoderFilter 
POST_BUILD COMMAND 
regsvr32 /s \"$(TargetPath)\"
)
ENDIF (REGISTER_DS_FILTERS)
