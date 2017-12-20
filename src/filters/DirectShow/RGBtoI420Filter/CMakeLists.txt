# CMakeLists.txt for <RGBtoI420Filter> 

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")

INCLUDE_DIRECTORIES(
${artistIncludes}
${artist_SOURCE_DIR}/../include/artist
${artistCodecIncludes}
${artistUtilIncludes}
)

# Lib directories
LINK_DIRECTORIES(
${artistLink}
)

SET(FLT_HDRS
resource.h
RGBtoI420Properties.h
RGBtoI420Filter.h
stdafx.h
VersionInfo.h
)

SET(FLT_SRCS 
DLLSetup.cpp
RGBtoI420Filter.cpp
RGBtoI420Filter.def
RGBtoI420Filter.rc
stdafx.cpp
)

ADD_LIBRARY(
RGBtoI420Filter SHARED ${FLT_SRCS} ${FLT_HDRS})

TARGET_LINK_LIBRARIES (
RGBtoI420Filter
${DsFilterLibs}
RtvcImageUtils
) 

INSTALL(
  TARGETS RGBtoI420Filter
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION bin
)

IF (REGISTER_DS_FILTERS)
ADD_CUSTOM_COMMAND(
TARGET RGBtoI420Filter 
POST_BUILD COMMAND 
regsvr32 /s \"$(TargetPath)\"
)
ENDIF(REGISTER_DS_FILTERS)
