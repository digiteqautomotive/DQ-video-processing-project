# CMakeLists.txt for <I420ToRGBFilter> 

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
stdafx.h
I420ToRGBFilter.h
I420ToRGBProperties.h
VersionInfo.h
)

SET(FLT_SRCS 
DLLSetup.cpp
stdafx.cpp
I420ToRGBFilter.cpp
I420ToRGBFilter.def
I420ToRGBFilter.rc
)

ADD_LIBRARY(
I420ToRGBFilter SHARED ${FLT_SRCS} ${FLT_HDRS})

TARGET_LINK_LIBRARIES (
I420ToRGBFilter
${DsFilterLibs}
RtvcImageUtils
) 

INSTALL(
  TARGETS I420ToRGBFilter
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION bin
)

IF (REGISTER_DS_FILTERS)
ADD_CUSTOM_COMMAND(
TARGET I420ToRGBFilter 
POST_BUILD COMMAND 
regsvr32 /s \"$(TargetPath)\"
)
ENDIF(REGISTER_DS_FILTERS)
