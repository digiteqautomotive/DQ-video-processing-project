# CMakeLists.txt for <H264EncoderFilter> 

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS")

if (USE_GLOBAL_VPP)
message("Using global VPP lib to build H264v2")
set(VPP_DIR
${GLOBAL_VPP_DIR}
)
else (USE_GLOBAL_VPP)
message("Using local VPP lib to build H264v2")
set(VPP_DIR
${vpp_SOURCE_DIR}/libs/VppLib
)
endif(USE_GLOBAL_VPP)

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
H264EncoderFilter.h
H264EncoderProperties.h
resource.h
stdafx.h
VersionInfo.h
)

SET(FLT_SRCS 
DLLSetup.cpp
H264EncoderFilter.cpp
H264EncoderFilter.def
H264EncoderFilter.rc
stdafx.cpp
)

ADD_LIBRARY(
H264EncoderFilter SHARED ${FLT_SRCS} ${FLT_HDRS})

TARGET_LINK_LIBRARIES (
H264EncoderFilter
baseclasses
strmiids
RtvcDirectShowExt
H264v2
winmm
comctl32
) 

INSTALL(
  TARGETS H264EncoderFilter
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION bin
)

IF (REGISTER_DS_FILTERS)
ADD_CUSTOM_COMMAND(
TARGET H264EncoderFilter 
POST_BUILD COMMAND 
regsvr32 /s \"$(TargetPath)\"
)
ENDIF(REGISTER_DS_FILTERS)
